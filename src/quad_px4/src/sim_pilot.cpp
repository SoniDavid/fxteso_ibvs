// The safety pilot, for SITL only: indoors PX4 has no GNSS aiding and cannot fly AUTO_TAKEOFF,
// so a human takes off in Altitude and hands over. Altitude is selected BEFORE arming, since
// AUTO_LOITER is position-controlled and the arm would be denied. It keeps flying rather than
// centring the sticks - a centred stick commands level attitude, not position hold - closing a
// horizontal loop on /quad_state ground truth, which no part of the IBVS chain reads.
// The stick stream never stops, because PX4 treats a gap as RC loss.
// Stamps come off PX4's clock: hrt_absolute_time() here is wall clock, and ROS-stamped
// samples look ~56 years old and are silently discarded.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include "quad_px4/px4_topic.hpp"

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

using px4_msgs::msg::ManualControlSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// PX4 custom main modes, as px4_offboard_bridge already uses them: 2 is ALTCTL.
static constexpr float MAIN_MODE_ALTCTL = 2.f;

enum class Phase
{
	WaitEkf,     // no valid height yet
	Altitude,    // ALTCTL requested, waiting for it - BEFORE arming, see below
	Arm,
	Climb,       // throttle up until the handover height
	Hold,        // sticks centred; PX4 holds height and the offboard bridge takes over
};

static const char *phaseName(Phase p)
{
	switch (p)
	{
	case Phase::WaitEkf:  return "WaitEkf";
	case Phase::Arm:      return "Arm";
	case Phase::Altitude: return "Altitude";
	case Phase::Climb:    return "Climb";
	case Phase::Hold:     return "Hold";
	}
	return "?";
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("sim_pilot");

	const double rate_hz = node->declare_parameter<double>("rate", 50.0);
	// Where the pilot stops climbing and hands over. The offboard bridge waits for the same
	// height, so this must not be lower than its takeoff_altitude or it never switches.
	const double takeoff_altitude = node->declare_parameter<double>("takeoff_altitude", 1.5);
	// ALTCTL throttle is a climb-RATE demand, so trim proportionally rather than cut and coast.
	// 2.0 clears PX4's stick deadzone (MPC_HOLD_DZ); softer gains stall short of the target.
	const double kp_alt = node->declare_parameter<double>("pilot_kp_alt", 2.0);
	// Ceiling on that demand, so the climb is brisk but not a launch.
	const double climb_stick = node->declare_parameter<double>("climb_stick", 0.35);
	// Close enough to call it handed over - logging only; the loop keeps trimming either way.
	const double lead = node->declare_parameter<double>("climb_lead", 0.05);
	const double command_retry = node->declare_parameter<double>("command_retry", 1.0);
	// Horizontal loop, stick units per metre and per m/s. The stick is a TILT demand, so this
	// is an attitude loop over position: damping does most of the work.
	const double kp = node->declare_parameter<double>("pilot_kp", 0.60);
	const double kd = node->declare_parameter<double>("pilot_kd", 0.50);
	const double stick_limit = node->declare_parameter<double>("pilot_stick_limit", 0.35);
	// The pilot points the nose as well as holding the spot. ibvs_gate wants |qpsi| <= 0.15 rad
	// before it will hand over, and nothing else was steering yaw - it sat at -0.293.
	const double kp_yaw = node->declare_parameter<double>("pilot_kp_yaw", 2.0);

	const rclcpp::QoS px4In = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
	using quad_px4::px4Topic;

	auto manualPub = node->create_publisher<ManualControlSetpoint>(
		px4Topic<ManualControlSetpoint>("/fmu/in/manual_control_input"), 10);
	auto commandPub = node->create_publisher<VehicleCommand>(
		px4Topic<VehicleCommand>("/fmu/in/vehicle_command"), 10);

	uint8_t nav_state = 0, arming_state = 0;
	bool have_status = false;
	// PX4's clock, and the local time it was read at, so the two can be related.
	uint64_t px4_stamp = 0;
	rclcpp::Time px4_stamp_local(0, 0, RCL_ROS_TIME);
	auto statusSub = node->create_subscription<VehicleStatus>(
		px4Topic<VehicleStatus>("/fmu/out/vehicle_status"), px4In,
		[&](const VehicleStatus::ConstSharedPtr s)
		{
			nav_state = s->nav_state;
			arming_state = s->arming_state;
			px4_stamp = s->timestamp;
			px4_stamp_local = node->now();
			have_status = true;
		});

	bool z_ok = false;
	double altitude = 0.0;
	auto posSub = node->create_subscription<VehicleLocalPosition>(
		px4Topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4In,
		[&](const VehicleLocalPosition::ConstSharedPtr p)
		{
			z_ok = p->z_valid;
			if (p->z_valid)
				altitude = -p->z;      // NED: down is positive
		});

	// PX4's own time base, advanced by however long ago we last heard from it.
	// Ground truth, in the same NED workspace frame gz_state_adapter publishes: gz is ENU with an
	// FLU body, so (x, -y, -z) on both the world position and the body-frame twist.
	bool have_truth = false;
	double quad_n = 0, quad_e = 0, yaw_ned = 0, v_fwd = 0, v_right = 0;
	auto truthSub = node->create_subscription<nav_msgs::msg::Odometry>(
		"quad_state", 10,
		[&](const nav_msgs::msg::Odometry::ConstSharedPtr odom)
		{
			const auto &p = odom->pose.pose.position;
			quad_n = p.x;
			quad_e = -p.y;

			tf2::Quaternion q;
			tf2::fromMsg(odom->pose.pose.orientation, q);
			double roll, pitch, yaw;
			tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
			yaw_ned = -yaw;

			// nav_msgs twist is body frame; flipped it is body FRD, so x forward and y right.
			v_fwd = odom->twist.twist.linear.x;
			v_right = -odom->twist.twist.linear.y;
			have_truth = true;
		});

	bool have_target = false;
	double tgt_n = 0, tgt_e = 0, tgt_yaw = 0;
	auto tgtSub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 10,
		[&](const geometry_msgs::msg::Vector3::ConstSharedPtr t)
		{
			tgt_n = t->x;
			tgt_e = t->y;
			have_target = true;
		});
	auto tgtYawSub = node->create_subscription<std_msgs::msg::Float64>(
		"tgt_yaw", 10,
		[&](const std_msgs::msg::Float64::ConstSharedPtr y) { tgt_yaw = y->data; });

	auto stamp = [&]() -> uint64_t
	{
		if (px4_stamp == 0)
			return 0;
		const double since = (node->now() - px4_stamp_local).seconds();
		return px4_stamp + static_cast<uint64_t>(since * 1e6);
	};

	auto sendCommand = [&](uint32_t command, float p1 = 0.f, float p2 = 0.f)
	{
		VehicleCommand msg;
		msg.timestamp = stamp();
		msg.command = command;
		msg.param1 = p1;
		msg.param2 = p2;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		commandPub->publish(msg);
	};

	// Roll and pitch holding station over the target, in the stick's convention: pitch positive
	// forward, roll positive right.
	auto holdSticks = [&](double &roll_out, double &pitch_out, double &yaw_out)
	{
		roll_out = pitch_out = yaw_out = 0.0;
		if (!have_truth || !have_target)
			return;
		const double e_n = tgt_n - quad_n;
		const double e_e = tgt_e - quad_e;
		const double c = std::cos(yaw_ned), s_ = std::sin(yaw_ned);
		const double e_fwd = c * e_n + s_ * e_e;
		const double e_right = -s_ * e_n + c * e_e;
		pitch_out = std::clamp(kp * e_fwd - kd * v_fwd, -stick_limit, stick_limit);
		roll_out = std::clamp(kp * e_right - kd * v_right, -stick_limit, stick_limit);

		// Positive yaw stick is a clockwise rate seen from above, which is +yaw in NED.
		double e_yaw = tgt_yaw - yaw_ned;
		while (e_yaw > M_PI)  e_yaw -= 2.0 * M_PI;
		while (e_yaw < -M_PI) e_yaw += 2.0 * M_PI;
		yaw_out = std::clamp(kp_yaw * e_yaw, -stick_limit, stick_limit);
	};

	auto publishSticks = [&](double throttle, double roll = 0.0, double pitch = 0.0,
	                         double yaw = 0.0)
	{
		ManualControlSetpoint msg;
		msg.timestamp = stamp();
		msg.timestamp_sample = msg.timestamp;
		msg.valid = true;
		msg.data_source = ManualControlSetpoint::SOURCE_MAVLINK_0;
		msg.roll = static_cast<float>(std::clamp(roll, -1.0, 1.0));
		msg.pitch = static_cast<float>(std::clamp(pitch, -1.0, 1.0));
		msg.yaw = static_cast<float>(std::clamp(yaw, -1.0, 1.0));
		msg.throttle = static_cast<float>(std::clamp(throttle, -1.0, 1.0));
		msg.sticks_moving = false;     // true would be an override request and would drop OFFBOARD
		manualPub->publish(msg);
	};

	Phase phase = Phase::WaitEkf;
	rclcpp::Time last_command(0, 0, RCL_ROS_TIME);

	auto enter = [&](Phase next)
	{
		RCLCPP_INFO(node->get_logger(), "%s -> %s", phaseName(phase), phaseName(next));
		phase = next;
		last_command = rclcpp::Time(0, 0, RCL_ROS_TIME);
	};

	auto dueForCommand = [&]()
	{
		const rclcpp::Time now = node->now();
		if (last_command.nanoseconds() != 0 && (now - last_command).seconds() < command_retry)
			return false;
		last_command = now;
		return true;
	};

	RCLCPP_INFO(node->get_logger(),
	            "SITL safety pilot: arm, Altitude, climb to %.2f m, then hold. Stick stream at "
	            "%.0f Hz - stopping it is an RC loss, which is the real failsafe.",
	            takeoff_altitude, rate_hz);

	fxteso::SimRate loop_rate(node, rate_hz);

	while (rclcpp::ok())
	{
		double throttle = 0.0;

		switch (phase)
		{
		case Phase::WaitEkf:
			if (have_status && z_ok)
				enter(Phase::Altitude);
			break;

		case Phase::Altitude:
			// ALTCTL first, then arm: AUTO_LOITER is position-controlled and the arm is denied.
			if (nav_state == VehicleStatus::NAVIGATION_STATE_ALTCTL)
				enter(Phase::Arm);
			else if (dueForCommand())
				sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, MAIN_MODE_ALTCTL);
			break;

		case Phase::Arm:
			if (arming_state == VehicleStatus::ARMING_STATE_ARMED)
				enter(Phase::Climb);
			else if (dueForCommand())
				sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
			break;

		case Phase::Climb:
			throttle = std::clamp(kp_alt * (takeoff_altitude - altitude),
			                      -climb_stick, climb_stick);
			if (altitude >= takeoff_altitude - lead)
			{
				RCLCPP_INFO(node->get_logger(), "At %.2f m - holding.", altitude);
				enter(Phase::Hold);
			}
			break;

		case Phase::Hold:
			// Keep trimming height too: if the bridge ever drops the stream PX4 comes back
			// here, and the pilot must still be flying.
			throttle = std::clamp(kp_alt * (takeoff_altitude - altitude),
			                      -climb_stick, climb_stick);
			break;
		}

		// Fly it, from the moment it is armed onward - including through Hold, where the offboard
		// bridge may take over and hand back.
		double roll_stick = 0.0, pitch_stick = 0.0, yaw_stick = 0.0;
		if (phase == Phase::Climb || phase == Phase::Hold)
			holdSticks(roll_stick, pitch_stick, yaw_stick);

		if (have_status)
			publishSticks(throttle, roll_stick, pitch_stick, yaw_stick);
		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}

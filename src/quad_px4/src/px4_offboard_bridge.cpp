// The setpoint sink for plant:=px4: pos_ctrl's desired attitude and collective thrust out as
// OFFBOARD setpoints, plus the arm/takeoff/handover sequence. Only units and frames change
// here - PX4's mc_att_control and mc_rate_control replace att_ctrl.
//
//   WAIT_EKF -> ARM -> TAKEOFF -> (loiter) -> STREAMING -> OFFBOARD
//
// The first desired_attitude triggers the OFFBOARD switch, which is sufficient on its own:
// ibvs_gate has already gated pos_ctrl on a marker lock.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include "quad_px4/px4_topic.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float64.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>
#include <cmath>

using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::VehicleAttitudeSetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

enum class Phase
{
	WaitEkf,     // EKF2 has not converged; do nothing
	Arm,         // arm command sent, waiting for ARMING_STATE_ARMED
	Takeoff,     // takeoff commanded, waiting to reach loiter at altitude
	Streaming,   // OffboardControlMode flowing, waiting for pos_ctrl
	Offboard,    // servoing
};

static const char *phaseName(Phase p)
{
	switch (p)
	{
	case Phase::WaitEkf:   return "WaitEkf";
	case Phase::Arm:       return "Arm";
	case Phase::Takeoff:   return "Takeoff";
	case Phase::Streaming: return "Streaming";
	case Phase::Offboard:  return "Offboard";
	}
	return "?";
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("px4_offboard_bridge");

	// Matches pos_ctrl's own loop, so one desired_attitude produces one setpoint.
	const double rate_hz = node->declare_parameter<double>("rate", 50.0);

	// Newton -> normalized, mirroring PX4's own linearization about hover. Exact only at hover,
	// deliberately: that is the map that survives to hardware. Nothing downstream corrects a bad
	// anchor, so hover_thrust must equal MPC_THR_HOVER in the airframe.
	const double hover_thrust = node->declare_parameter<double>("hover_thrust", 0.716);
	const double mass = node->declare_parameter<double>("mass", 2.0);
	const double gravity = node->declare_parameter<double>("gravity", 9.81);

	// Losing vision hands the aircraft back to PX4: dropping the stream triggers COM_OBL_RC_ACT.
	const double setpoint_timeout = node->declare_parameter<double>("setpoint_timeout", 0.3);
	// PX4 wants the stream established before it will accept the mode switch.
	const double stream_before_switch = node->declare_parameter<double>("stream_before_switch", 1.5);
	// Retry interval for arm and mode commands, which PX4 may reject while still settling.
	const double command_retry = node->declare_parameter<double>("command_retry", 1.0);
	// Must match MIS_TAKEOFF_ALT in the airframe, and so zD in aibvs_pos_ctrl.cpp.
	const double takeoff_altitude = node->declare_parameter<double>("takeoff_altitude", 2.5);
	const double altitude_tolerance = node->declare_parameter<double>("altitude_tolerance", 0.4);

	// The mirror of px4_state_adapter's frame_yaw_offset, and MUST be the same number: that one
	// rotates into the workspace frame, this one rotates back out.
	const double frame_yaw = node->declare_parameter<double>("frame_yaw_offset", 0.0);
	const tf2::Quaternion qFrameInv(tf2::Vector3(0, 0, 1), -frame_yaw);

	const double weight = mass * gravity;

	const rclcpp::QoS px4In = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

	// px4Topic appends the message's _vN where PX4's DDS client does; see px4_topic.hpp.
	using quad_px4::px4Topic;

	auto offboardModePub = node->create_publisher<OffboardControlMode>(
		px4Topic<OffboardControlMode>("/fmu/in/offboard_control_mode"), 10);
	auto attSpPub = node->create_publisher<VehicleAttitudeSetpoint>(
		px4Topic<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint"), 10);
	auto commandPub = node->create_publisher<VehicleCommand>(
		px4Topic<VehicleCommand>("/fmu/in/vehicle_command"), 10);

	// --- inputs ---------------------------------------------------------------
	double roll_d = 0.0, pitch_d = 0.0, yaw_d = 0.0;
	double thrust_n = weight;          // hover until told otherwise
	rclcpp::Time last_setpoint(0, 0, RCL_ROS_TIME);
	bool have_setpoint = false;

	auto attSub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"desired_attitude", 1,
		[&](const geometry_msgs::msg::Quaternion::ConstSharedPtr d)
		{
			// pos_ctrl packs Euler angles plus the yaw rate into a Quaternion message, not a quaternion.
			// d->w is deliberately unused - see publishAttitudeSetpoint.
			roll_d = d->x;
			pitch_d = d->y;
			yaw_d = d->z;
			last_setpoint = node->now();
			have_setpoint = true;
		});

	auto thrustSub = node->create_subscription<std_msgs::msg::Float64>(
		"quad_thrust", 1,
		[&](const std_msgs::msg::Float64::ConstSharedPtr t) { thrust_n = t->data; });

	uint8_t nav_state = 0;
	uint8_t arming_state = 0;
	bool have_status = false;
	auto statusSub = node->create_subscription<VehicleStatus>(
		px4Topic<VehicleStatus>("/fmu/out/vehicle_status"), px4In,
		[&](const VehicleStatus::ConstSharedPtr s)
		{
			nav_state = s->nav_state;
			arming_state = s->arming_state;
			have_status = true;
		});

	bool ekf_ready = false;
	double altitude = 0.0;             // metres above the takeoff point; z is NED, so negated
	auto localPosSub = node->create_subscription<VehicleLocalPosition>(
		px4Topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4In,
		[&](const VehicleLocalPosition::ConstSharedPtr p)
		{
			ekf_ready = p->xy_valid && p->z_valid;
			if (p->z_valid)
				altitude = -p->z;
		});

	// --- helpers --------------------------------------------------------------
	auto stamp = [&]() { return static_cast<uint64_t>(node->now().nanoseconds() / 1000); };

	auto sendCommand = [&](uint32_t command, float p1 = 0.f, float p2 = 0.f, float p3 = 0.f)
	{
		VehicleCommand msg;
		msg.timestamp = stamp();
		msg.command = command;
		msg.param1 = p1;
		msg.param2 = p2;
		msg.param3 = p3;
		msg.target_system = 1;
		msg.target_component = 1;
		msg.source_system = 1;
		msg.source_component = 1;
		msg.from_external = true;
		commandPub->publish(msg);
	};

	auto publishOffboardMode = [&]()
	{
		OffboardControlMode msg;
		msg.timestamp = stamp();
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = true;
		msg.body_rate = false;
		msg.thrust_and_torque = false;
		msg.direct_actuator = false;
		offboardModePub->publish(msg);
	};

	auto publishAttitudeSetpoint = [&]()
	{
		VehicleAttitudeSetpoint msg;
		msg.timestamp = stamp();

		// pos_ctrl's Euler triple is in the workspace frame; rotate it back into PX4's.
		tf2::Quaternion qWs;
		qWs.setRPY(roll_d, pitch_d, yaw_d);
		const tf2::Quaternion q = qFrameInv * qWs;
		msg.q_d[0] = static_cast<float>(q.w());
		msg.q_d[1] = static_cast<float>(q.x());
		msg.q_d[2] = static_cast<float>(q.y());
		msg.q_d[3] = static_cast<float>(q.z());

		const double norm = std::clamp(hover_thrust * thrust_n / weight, 0.0, 1.0);
		msg.thrust_body[0] = 0.f;
		msg.thrust_body[1] = 0.f;
		msg.thrust_body[2] = static_cast<float>(-norm);   // body FRD: down is negative

		// No yaw_sp_move_rate: pos_ctrl already folds the rate into the yaw in q_d, so feeding
		// it again makes PX4 lead the command.
		msg.yaw_sp_move_rate = 0.f;
		attSpPub->publish(msg);
	};

	// --- state machine --------------------------------------------------------
	Phase phase = Phase::WaitEkf;
	bool seen_takeoff = false;
	rclcpp::Time last_command(0, 0, RCL_ROS_TIME);
	rclcpp::Time stream_since(0, 0, RCL_ROS_TIME);

	auto enter = [&](Phase next)
	{
		RCLCPP_INFO(node->get_logger(), "%s -> %s", phaseName(phase), phaseName(next));
		phase = next;
		last_command = rclcpp::Time(0, 0, RCL_ROS_TIME);
	};

	// True once enough time has passed to retry a command PX4 did not act on.
	auto dueForCommand = [&]()
	{
		const rclcpp::Time now = node->now();
		if (last_command.nanoseconds() != 0 &&
		    (now - last_command).seconds() < command_retry)
			return false;
		last_command = now;
		return true;
	};

	RCLCPP_INFO(node->get_logger(),
	            "Waiting for EKF2. Will arm, take off, then hand over to OFFBOARD on the first "
	            "desired_attitude. Thrust map: %.2f normalized at %.2f N.",
	            hover_thrust, weight);

	fxteso::SimRate loop_rate(node, rate_hz);

	while (rclcpp::ok())
	{
		const rclcpp::Time now = node->now();
		const bool setpoint_fresh =
			have_setpoint && (now - last_setpoint).seconds() < setpoint_timeout;

		switch (phase)
		{
		case Phase::WaitEkf:
			if (have_status && ekf_ready)
				enter(Phase::Arm);
			break;

		case Phase::Arm:
			if (arming_state == VehicleStatus::ARMING_STATE_ARMED)
				enter(Phase::Takeoff);
			else if (dueForCommand())
				sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f);
			break;

		case Phase::Takeoff:
			// AUTO_LOITER is also the resting state on the ground, so it proves nothing on its own:
			// wait for AUTO_TAKEOFF to be entered, then for the altitude to be reached.
			if (nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
			{
				seen_takeoff = true;
			}
			else if (!seen_takeoff)
			{
				if (dueForCommand())
					sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 4.f, 2.f);
				break;
			}

			if (seen_takeoff && altitude >= takeoff_altitude - altitude_tolerance)
			{
				RCLCPP_INFO(node->get_logger(), "Reached %.2f m", altitude);
				enter(Phase::Streaming);
			}
			break;

		case Phase::Streaming:
			// Legal to stream while still in loiter, and PX4 requires it before the switch.
			publishOffboardMode();
			if (stream_since.nanoseconds() == 0)
				stream_since = now;

			if (setpoint_fresh && (now - stream_since).seconds() >= stream_before_switch)
			{
				if (nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD)
					enter(Phase::Offboard);
				else if (dueForCommand())
					sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.f, 6.f);
			}
			break;

		case Phase::Offboard:
			if (!setpoint_fresh)
			{
				// Stop the stream rather than repeat a stale attitude; PX4 then fails safe.
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
				                     "desired_attitude stale for more than %.2f s - dropping the "
				                     "offboard stream and letting PX4 fail safe.",
				                     setpoint_timeout);
				break;
			}
			publishOffboardMode();
			publishAttitudeSetpoint();
			break;
		}

		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}

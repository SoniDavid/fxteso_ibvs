// The setpoint sink for plant:=px4: pos_ctrl's desired attitude and thrust out as OFFBOARD
// setpoints. auto: WAIT_EKF -> ARM -> TAKEOFF -> STREAMING -> OFFBOARD. pilot: WAIT_EKF ->
// WAIT_PILOT -> STREAMING -> OFFBOARD, where a human flies it up and this never arms.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include "quad_px4/px4_topic.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string>

using px4_msgs::msg::ManualControlSetpoint;
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
	WaitPilot,   // bringup:=pilot - a human arms and flies it to altitude; we touch nothing
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
	case Phase::WaitPilot: return "WaitPilot";
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
	const double hover_thrust = node->declare_parameter<double>("hover_thrust", 0.6461);
	const double mass = node->declare_parameter<double>("mass", 2.0);
	const double gravity = node->declare_parameter<double>("gravity", 9.81);

	// Losing vision hands the aircraft back to PX4: dropping the stream triggers COM_OBL_RC_ACT.
	const double setpoint_timeout = node->declare_parameter<double>("setpoint_timeout", 0.3);
	// PX4 wants the stream established before it will accept the mode switch.
	const double stream_before_switch = node->declare_parameter<double>("stream_before_switch", 1.5);
	// Retry interval for arm and mode commands, which PX4 may reject while still settling.
	const double command_retry = node->declare_parameter<double>("command_retry", 1.0);
	// "auto" arms and commands AUTO_TAKEOFF; "pilot" does neither and waits for the human.
	// Defaults to the passive one: a bare `ros2 run` must not arm an aircraft.
	const std::string bringup = node->declare_parameter<std::string>("bringup", "pilot");
	if (bringup != "auto" && bringup != "pilot")
	{
		RCLCPP_FATAL(node->get_logger(), "bringup:=%s must be 'auto' or 'pilot'.", bringup.c_str());
		return 1;
	}
	const bool pilot_bringup = (bringup == "pilot");

	// KNOWN HARMFUL, default false. PX4 recovers on its own; this re-arms OFFBOARD off the
	// heartbeat alone, while the loop is still blind. Kept only to reproduce E59.
	const bool offboard_recovery = node->declare_parameter<bool>("offboard_recovery", false);
	// So an intermittently visible target cannot flap the aircraft between Hold and OFFBOARD.
	const int max_recoveries = node->declare_parameter<int>("max_recoveries", 5);

	// Must match MIS_TAKEOFF_ALT in the airframe, and so zD in aibvs_pos_ctrl.cpp.
	const double takeoff_altitude = node->declare_parameter<double>("takeoff_altitude", 2.5);
	// 0.4 m fires the handover 0.4 m BELOW the target, mid-climb. Callers pass their own.
	const double altitude_tolerance = node->declare_parameter<double>("altitude_tolerance", 0.4);

	// Altitude alone is not consent: the handover also needs the RC aux switch or ~/handover,
	// and withdrawing it drops the stream. Defaults true - wait for a person.
	const bool require_consent = node->declare_parameter<bool>("require_consent", true);
	// 1..6 selects manual_control_setpoint.auxN; needs RC_MAP_AUXn. 0 = services only.
	const int consent_aux = node->declare_parameter<int>("consent_rc_aux", 0);
	const double consent_threshold = node->declare_parameter<double>("consent_rc_threshold", 0.5);
	if (consent_aux < 0 || consent_aux > 6)
	{
		RCLCPP_FATAL(node->get_logger(), "consent_rc_aux:=%d must be 0..6.", consent_aux);
		return 1;
	}

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
	// One-way latch: a pilot who has taken over must never have the aircraft grabbed back.
	bool pilot_has_it = false;
	auto statusSub = node->create_subscription<VehicleStatus>(
		px4Topic<VehicleStatus>("/fmu/out/vehicle_status"), px4In,
		[&](const VehicleStatus::ConstSharedPtr s)
		{
			nav_state = s->nav_state;
			arming_state = s->arming_state;
			// PX4's own flag. nav_state_user_intention cannot serve: it reads OFFBOARD from the
			// moment this node commands the mode, so it cannot tell the pilot from us.
			if (s->failsafe_and_user_took_over && !pilot_has_it)
			{
				pilot_has_it = true;
				RCLCPP_WARN(node->get_logger(),
				            "the pilot took the aircraft out of a failsafe - offboard recovery "
				            "is disabled for the rest of this flight.");
			}
			have_status = true;
		});

	bool ekf_ready = false;
	double altitude = 0.0;             // metres above the takeoff point; z is NED, so negated
	auto localPosSub = node->create_subscription<VehicleLocalPosition>(
		px4Topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4In,
		[&](const VehicleLocalPosition::ConstSharedPtr p)
		{
			// xy_valid needs horizontal aiding, which indoors never arrives; nothing downstream
			// uses horizontal position.
			ekf_ready = pilot_bringup ? p->z_valid : (p->xy_valid && p->z_valid);
			if (p->z_valid)
				altitude = -p->z;
		});

	// Two grants, either sufficient, both LIVE: withdrawing one takes the aircraft back.
	// Only meaningful under bringup:=pilot; auto has no human on the sticks to ask.
	const bool consent_required = require_consent && pilot_bringup;
	bool consent_rc = false;
	bool consent_service = false;
	auto consentGiven = [&]() { return !consent_required || consent_rc || consent_service; };

	rclcpp::Subscription<ManualControlSetpoint>::SharedPtr manualSub;
	if (consent_aux > 0)
	{
		manualSub = node->create_subscription<ManualControlSetpoint>(
			px4Topic<ManualControlSetpoint>("/fmu/out/manual_control_setpoint"), px4In,
			[&](const ManualControlSetpoint::ConstSharedPtr m)
			{
				const float aux[6] = {m->aux1, m->aux2, m->aux3, m->aux4, m->aux5, m->aux6};
				const float v = aux[consent_aux - 1];
				// An unmapped channel is NaN, which must read as "no".
				const bool now_rc = m->valid && std::isfinite(v) && v >= consent_threshold;
				if (now_rc != consent_rc)
					RCLCPP_WARN(node->get_logger(), "RC consent (aux%d = %.2f): %s",
					            consent_aux, v, now_rc ? "GRANTED" : "WITHDRAWN");
				consent_rc = now_rc;
			});
	}

	auto handoverSrv = node->create_service<std_srvs::srv::Trigger>(
		"~/handover",
		[&](const std_srvs::srv::Trigger::Request::SharedPtr,
		    std_srvs::srv::Trigger::Response::SharedPtr res)
		{
			consent_service = true;
			RCLCPP_WARN(node->get_logger(), "handover consent GRANTED by service.");
			res->success = true;
			res->message = "consent granted";
		});

	auto abortSrv = node->create_service<std_srvs::srv::Trigger>(
		"~/abort",
		[&](const std_srvs::srv::Trigger::Request::SharedPtr,
		    std_srvs::srv::Trigger::Response::SharedPtr res)
		{
			consent_service = false;
			RCLCPP_WARN(node->get_logger(), "handover consent WITHDRAWN by service.");
			res->success = true;
			res->message = "consent withdrawn";
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
	int recoveries = 0;   // offboard re-commands spent; see Phase::Offboard
	bool seen_takeoff = false;
	rclcpp::Time last_command(0, 0, RCL_ROS_TIME);
	rclcpp::Time stream_since(0, 0, RCL_ROS_TIME);

	auto enter = [&](Phase next)
	{
		RCLCPP_INFO(node->get_logger(), "%s -> %s", phaseName(phase), phaseName(next));
		phase = next;
		last_command = rclcpp::Time(0, 0, RCL_ROS_TIME);
		// Cleared on every transition: Streaming only initialises it when zero, so a re-entry
		// would inherit the first timestamp and switch mode before PX4 accepts it.
		stream_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
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

	if (pilot_bringup)
		RCLCPP_INFO(node->get_logger(),
		            "Waiting for EKF2. bringup:=pilot - this will NOT arm or take off. Offboard "
		            "is requested once the pilot is armed at %.2f m (+/- %.2f) %s. "
		            "Thrust map: %.2f normalized at %.2f N.",
		            takeoff_altitude, altitude_tolerance,
		            consent_required
		                ? (consent_aux > 0 ? "and consents by RC aux or ~/handover"
		                                   : "and consents by ~/handover")
		                : "(CONSENT NOT REQUIRED)",
		            hover_thrust, weight);
	else
		RCLCPP_INFO(node->get_logger(),
		            "Waiting for EKF2. bringup:=auto - will arm, take off, then hand over to "
		            "OFFBOARD on the first desired_attitude. Thrust map: %.2f normalized at %.2f N.",
		            hover_thrust, weight);

	fxteso::SimRate loop_rate(node, rate_hz);

	while (rclcpp::ok())
	{
		const rclcpp::Time now = node->now();
		const bool setpoint_fresh =
			have_setpoint && (now - last_setpoint).seconds() < setpoint_timeout;

		// Withdrawing consent aborts the same way a lock loss does: stop the stream.
		if ((phase == Phase::Streaming || phase == Phase::Offboard) && !consentGiven())
		{
			RCLCPP_ERROR(node->get_logger(),
			             "consent withdrawn - dropping the offboard stream, PX4 fails safe.");
			enter(Phase::WaitPilot);
			loop_rate.sleep();
			continue;
		}

		switch (phase)
		{
		case Phase::WaitEkf:
			if (have_status && ekf_ready)
				enter(pilot_bringup ? Phase::WaitPilot : Phase::Arm);
			break;

		case Phase::WaitPilot:
			// Passive: no arm, no mode command until the pilot is armed, at height, and
			// consenting. Streaming will not switch mode until setpoint_fresh anyway.
			if (arming_state == VehicleStatus::ARMING_STATE_ARMED &&
			    altitude >= takeoff_altitude - altitude_tolerance &&
			    consentGiven())
			{
				RCLCPP_INFO(node->get_logger(),
				            "Pilot has it at %.2f m, armed and consenting - requesting offboard.",
				            altitude);
				enter(Phase::Streaming);
			}
			else if (arming_state == VehicleStatus::ARMING_STATE_ARMED &&
			         altitude >= takeoff_altitude - altitude_tolerance)
			{
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
				                     "At %.2f m and armed, waiting for consent: flip the RC aux "
				                     "switch or call ~/handover.", altitude);
			}
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

				// Still in OFFBOARD means COM_OF_LOSS_T has not expired: nothing to recover from.
				if (nav_state != VehicleStatus::NAVIGATION_STATE_OFFBOARD)
				{
					if (!offboard_recovery)
						RCLCPP_WARN_THROTTLE(
							node->get_logger(), *node->get_clock(), 5000,
							"PX4 has failed safe and offboard_recovery is off - this run is over "
							"even if the markers come back.");
					else if (pilot_has_it)
						RCLCPP_WARN_THROTTLE(
							node->get_logger(), *node->get_clock(), 5000,
							"PX4 has failed safe but the pilot has the aircraft - not recovering.");
					else if (recoveries >= max_recoveries)
						RCLCPP_WARN_THROTTLE(
							node->get_logger(), *node->get_clock(), 5000,
							"PX4 has failed safe and max_recoveries (%d) is spent - not recovering.",
							max_recoveries);
					else
					{
						++recoveries;
						RCLCPP_WARN(node->get_logger(),
						            "PX4 failed safe out of OFFBOARD (nav_state %u). Re-streaming "
						            "to recover, attempt %d of %d.",
						            nav_state, recoveries, max_recoveries);
						enter(Phase::Streaming);
					}
				}
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

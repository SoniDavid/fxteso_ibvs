// Vicon -> EKF2. A mocap PoseStamped becomes VehicleOdometry on /fmu/in/vehicle_visual_odometry,
// the mirror of px4_state_adapter. Published only while the loop does NOT have the aircraft:
// under aiding_policy:=gated the stream stops at the handover, so no ground truth reaches the
// estimator during the window the run is scored on.
#include <rclcpp/rclcpp.hpp>
#include "quad_px4/px4_topic.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

using px4_msgs::msg::EstimatorStatusFlags;
using px4_msgs::msg::VehicleOdometry;
using px4_msgs::msg::VehicleStatus;
using quad_px4::px4Topic;

// EKF2 drops a sample older than EV_MAX_INTERVAL (EKF/common.h), so a lag beyond it is useless.
static constexpr int64_t kMaxSampleLagUs = 200000;
// EKF2 rejects the attitude outright unless |1 - |q|| <= 1e-5 (EKF2.cpp, norm_in_tolerance).
static constexpr double kQuatNormTolerance = 1e-6;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("vicon_px4_bridge");

	const std::string vicon_topic =
		node->declare_parameter<std::string>("vicon_topic", "/vicon/quad/quad");
	const double rate = node->declare_parameter<double>("rate", 50.0);
	// Older than this and the stream stops rather than repeating the last pose: a frozen sample
	// reads to EKF2 as a perfect measurement and fights the IMU.
	const double max_pose_age = node->declare_parameter<double>("max_pose_age", 0.15);

	// DIAGNOSTIC, default off. An Odometry whose BODY-frame twist is published to EKF2 as EV
	// velocity with position left NaN, so EKF2 gets a velocity reference and NO position one -
	// which is what an optical flow sensor supplies. Exists to answer, before any flow hardware
	// is bought, whether velocity alone bounds EKF2's tilt error: every result showing aiding
	// works (E85, E88) used mocap POSITION. Pair with EKF2_EV_CTRL=4 (bit 2, 3D velocity).
	// Ground truth in, so it is contaminated by construction and is never a GPS-denied result.
	const std::string velocity_topic =
		node->declare_parameter<std::string>("velocity_topic", "");
	const double velocity_sd = node->declare_parameter<double>("velocity_sd", 0.05);

	// header: the Vicon receiver's stamp, which needs chrony between it and this machine.
	// receipt: this node's arrival time, leaving the transport latency to EKF2_EV_DELAY.
	const std::string stamp_source =
		node->declare_parameter<std::string>("stamp_source", "header");
	// enu: the Vicon global frame is Z-up right-handed, the usual wand calibration.
	const std::string vicon_frame = node->declare_parameter<std::string>("vicon_frame", "enu");
	// flu: the Vicon object template's axes are x-forward y-left z-up. frd needs no body rotation.
	const std::string body_frame = node->declare_parameter<std::string>("body_frame", "flu");
	// Rotates the Vicon world onto the workspace datum, about down. EV yaw makes EKF2's heading
	// relative to whatever is fed here, so px4_state_adapter's frame_yaw_offset stays 0 while aiding.
	const double yaw_offset = node->declare_parameter<double>("yaw_offset", 0.0);
	// Vicon origin -> workspace origin, NED metres, applied after the rotation.
	const std::vector<double> position_offset =
		node->declare_parameter<std::vector<double>>("position_offset", {0.0, 0.0, 0.0});

	// gated: aided iff the loop does not have the aircraft. always: bring-up and shadow flights.
	// oneshot: until the first handover, then never. manual: ~/vicon_aiding only.
	const std::string aiding_policy =
		node->declare_parameter<std::string>("aiding_policy", "gated");
	// Hysteresis on resume only; the cut is immediate. Stops a brief OFFBOARD blip flapping the
	// stream, because every resume costs an EKF2 position reset.
	const double resume_delay = node->declare_parameter<double>("resume_delay", 0.5);
	// px4_offboard_bridge raises this when it decides to command OFFBOARD, which leads PX4's
	// nav_state by stream_before_switch. Without it the cut lags the handover by EKF2's 0.4 s timeout.
	const std::string handover_topic = node->declare_parameter<std::string>(
		"handover_topic", "/px4_offboard_bridge/offboard_requested");
	// A silent VehicleStatus freezes nav_state at its last value. Frozen at OFFBOARD that would
	// withhold aiding for the rest of the flight, so a stale status reads as "not OFFBOARD".
	const double status_timeout = node->declare_parameter<double>("status_timeout", 1.0);

	// A wrong vicon_frame/body_frame/yaw_offset is SILENT: a rotated but self-consistent frame
	// makes EKF2 re-anchor to it and report ~zero innovation. The only witness is EKF2's own
	// attitude, which is why EV yaw is deliberately not fused (EKF2_EV_CTRL=1).
	const bool frame_check = node->declare_parameter<bool>("frame_check", true);
	// 30 deg, not 15: the smallest gross frame error is 90 deg, while EKF2's mag heading carries a
	// real bias of a few degrees. This separates the two with margin at both ends.
	const double max_attitude_error =
		node->declare_parameter<double>("max_attitude_error", 30.0);
	// EKF2's heading still jumps just after cs_yaw_align (108.9 deg, then 2.3 deg 20 ms later), so
	// the check waits until alignment has held this long.
	const double frame_check_settle = node->declare_parameter<double>("frame_check_settle", 1.0);

	// Reported in the message, but EKF2_EV_NOISE_MD:=1 makes PX4 use EKF2_EVP/EVA_NOISE instead.
	const double position_sd = node->declare_parameter<double>("position_sd", 0.02);
	const double orientation_sd = node->declare_parameter<double>("orientation_sd", 0.05);

	if (vicon_frame != "enu" && vicon_frame != "ned")
	{
		RCLCPP_FATAL(node->get_logger(), "vicon_frame:=%s must be enu or ned.", vicon_frame.c_str());
		return 1;
	}
	if (body_frame != "flu" && body_frame != "frd")
	{
		RCLCPP_FATAL(node->get_logger(), "body_frame:=%s must be flu or frd.", body_frame.c_str());
		return 1;
	}
	if (stamp_source != "header" && stamp_source != "receipt")
	{
		RCLCPP_FATAL(node->get_logger(), "stamp_source:=%s must be header or receipt.",
		             stamp_source.c_str());
		return 1;
	}
	if (aiding_policy != "gated" && aiding_policy != "always" && aiding_policy != "oneshot" &&
	    aiding_policy != "manual")
	{
		RCLCPP_FATAL(node->get_logger(),
		             "aiding_policy:=%s must be gated, always, oneshot or manual.",
		             aiding_policy.c_str());
		return 1;
	}
	if (position_offset.size() != 3)
	{
		RCLCPP_FATAL(node->get_logger(), "position_offset needs three elements, got %zu.",
		             position_offset.size());
		return 1;
	}

	// ENU -> NED is (x,y,z) -> (y,x,-z), a proper rotation of pi about (1,1,0)/sqrt(2).
	const tf2::Quaternion qWorld = vicon_frame == "enu"
		? tf2::Quaternion(tf2::Vector3(M_SQRT1_2, M_SQRT1_2, 0.0), M_PI)
		: tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
	// FLU -> FRD is pi about body x. Applied on the right: it rebases the body axes, not the world.
	const tf2::Quaternion qBody = body_frame == "flu"
		? tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), M_PI)
		: tf2::Quaternion(0.0, 0.0, 0.0, 1.0);
	const tf2::Quaternion qYaw(tf2::Vector3(0, 0, 1), yaw_offset);
	// One rotation from the Vicon world to the workspace NED, so position and attitude cannot disagree.
	const tf2::Quaternion qToNed = qYaw * qWorld;

	// The system clock, never node->now(): under use_sim_time the node clock is Gazebo's, and PX4
	// translates an inbound stamp against the uXRCE-DDS agent host's OS clock. sim_pilot.cpp records
	// what a sim-time stamp costs - the sample looks ~56 years old and is discarded in silence.
	rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);
	auto sysNowUs = [&]() { return sys_clock.now().nanoseconds() / 1000; };

	auto odomPub = node->create_publisher<VehicleOdometry>(
		px4Topic<VehicleOdometry>("/fmu/in/vehicle_visual_odometry"), 10);
	// Latched: the bag needs to know exactly when aiding was on, to exclude the resume transients.
	auto aidingPub = node->create_publisher<std_msgs::msg::Bool>(
		"~/aiding_active", rclcpp::QoS(1).transient_local());

	geometry_msgs::msg::PoseStamped pose;
	bool have_pose = false;
	int64_t rx_us = 0;

	// BEST_EFFORT takes from a RELIABLE publisher too, so the receiver's QoS cannot lock us out.
	auto poseSub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
		vicon_topic, rclcpp::SensorDataQoS(),
		[&](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
		{
			pose = *msg;
			rx_us = sysNowUs();
			have_pose = true;
		});

	// uXRCE-DDS publishes best-effort; a reliable subscription silently receives nothing.
	const rclcpp::QoS px4In = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

	uint8_t nav_state = 0;
	int64_t status_us = 0;
	auto statusSub = node->create_subscription<VehicleStatus>(
		px4Topic<VehicleStatus>("/fmu/out/vehicle_status"), px4In,
		[&](const VehicleStatus::ConstSharedPtr s)
		{
			nav_state = s->nav_state;
			status_us = sysNowUs();
		});

	// EKF2's own attitude, the independent witness the frame guard checks against.
	tf2::Quaternion qEkf(0, 0, 0, 1);
	bool have_ekf_att = false;
	// Until EKF2 has aligned its heading the witness is worthless - comparing against it fires a
	// FATAL that is not fatal, which is the fastest way to teach someone to ignore FATALs.
	bool yaw_aligned = false;
	int64_t yaw_aligned_since_us = 0;
	auto flagsSub = node->create_subscription<EstimatorStatusFlags>(
		px4Topic<EstimatorStatusFlags>("/fmu/out/estimator_status_flags"), px4In,
		[&](const EstimatorStatusFlags::ConstSharedPtr f)
		{
			yaw_aligned = f->cs_yaw_align;
			if (!yaw_aligned)
				yaw_aligned_since_us = 0;
			else if (yaw_aligned_since_us == 0)
				yaw_aligned_since_us = sysNowUs();
		});
	// FLU body -> FRD body is the same pi-about-x the attitude uses; no pose needed, which is
	// why the velocity arm cannot inherit a pose-frame error.
	std::array<float, 3> ev_vel{};
	bool have_vel = false;
	int64_t vel_rx_us = 0;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr velSub;
	if (!velocity_topic.empty())
	{
		velSub = node->create_subscription<nav_msgs::msg::Odometry>(
			velocity_topic, rclcpp::SensorDataQoS(),
			[&](const nav_msgs::msg::Odometry::ConstSharedPtr m)
			{
				const auto &v = m->twist.twist.linear;
				ev_vel = {static_cast<float>(v.x), static_cast<float>(-v.y),
				          static_cast<float>(-v.z)};
				vel_rx_us = sysNowUs();
				have_vel = true;
			});
		RCLCPP_WARN(node->get_logger(),
		            "velocity_topic:=%s - publishing EV VELOCITY (body FRD) with position NaN. "
		            "This is a DIAGNOSTIC arm; set EKF2_EV_CTRL=4 to fuse it.",
		            velocity_topic.c_str());
	}

	auto odomSub = node->create_subscription<VehicleOdometry>(
		px4Topic<VehicleOdometry>("/fmu/out/vehicle_odometry"), px4In,
		[&](const VehicleOdometry::ConstSharedPtr o)
		{
			if (!std::isfinite(o->q[0]))
				return;
			// px4_msgs orders (w,x,y,z); tf2 takes (x,y,z,w).
			qEkf = tf2::Quaternion(o->q[1], o->q[2], o->q[3], o->q[0]);
			have_ekf_att = true;
		});

	bool handover_requested = false;
	auto handoverSub = node->create_subscription<std_msgs::msg::Bool>(
		handover_topic, rclcpp::QoS(1).transient_local(),
		[&](const std_msgs::msg::Bool::ConstSharedPtr b) { handover_requested = b->data; });

	bool manual_enable = true;
	auto aidingSrv = node->create_service<std_srvs::srv::SetBool>(
		"~/vicon_aiding",
		[&](const std_srvs::srv::SetBool::Request::SharedPtr req,
		    std_srvs::srv::SetBool::Response::SharedPtr res)
		{
			manual_enable = req->data;
			RCLCPP_WARN(node->get_logger(), "Vicon aiding %s by service.",
			            manual_enable ? "ENABLED" : "DISABLED");
			res->success = true;
			res->message = manual_enable ? "aiding enabled" : "aiding disabled";
		});

	bool aiding = false, announced = false, ever_wanted = false, warned_clock = false;
	bool frame_validated = !frame_check, frame_failed = false;
	uint8_t reset_counter = 0;
	int64_t wanted_us = 0;
	// Age statistics, reported every 5 s so EKF2_EV_DELAY is set from data rather than guessed.
	int64_t age_sum = 0, age_max = 0, age_n = 0, report_us = 0, stale_drops = 0;

	auto setAiding = [&](bool next)
	{
		if (next == aiding && announced)
			return;
		// A resumed stream is a new EV source as far as EKF2 is concerned; the bump makes it
		// reset to vision deliberately rather than fight the gap.
		if (next && announced)
			++reset_counter;
		aiding = next;
		announced = true;

		std_msgs::msg::Bool msg;
		msg.data = aiding;
		aidingPub->publish(msg);
		RCLCPP_WARN(node->get_logger(), "Vicon aiding %s%s", aiding ? "ON" : "OFF",
		            aiding ? "" : " - EKF2 drops EV fusion ~0.4 s from now.");
	};

	auto timer = node->create_wall_timer(
		std::chrono::duration<double>(1.0 / rate),
		[&]()
		{
			const int64_t now_us = sysNowUs();

			// The loop has the aircraft, or px4_offboard_bridge is about to take it.
			const bool status_fresh =
				status_us != 0 && (now_us - status_us) < static_cast<int64_t>(status_timeout * 1e6);
			if (!status_fresh && status_us != 0)
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
				                     "No VehicleStatus for %.1f s - treating nav_state as unknown "
				                     "and aiding, rather than trusting a frozen OFFBOARD.",
				                     (now_us - status_us) * 1e-6);
			const bool loop_wants =
				(status_fresh && nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD) ||
				handover_requested;
			if (loop_wants)
			{
				ever_wanted = true;
				wanted_us = now_us;
			}

			bool want_aiding;
			if (aiding_policy == "always")
				want_aiding = true;
			else if (aiding_policy == "manual")
				want_aiding = manual_enable;
			else if (aiding_policy == "oneshot")
				want_aiding = !ever_wanted;
			else
				want_aiding = !loop_wants &&
				              (now_us - wanted_us) >= static_cast<int64_t>(resume_delay * 1e6);

			setAiding(want_aiding);
			if (!aiding)
				return;

			// Nothing before the first pose: a default-constructed quaternion is {0,0,0,0}, which
			// EKF2 rejects as zero-norm - and which both reference implementations publish.
			if (!have_pose)
			{
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
				                     "No pose on %s yet - publishing nothing.", vicon_topic.c_str());
				return;
			}

			if (now_us - rx_us > static_cast<int64_t>(max_pose_age * 1e6))
			{
				++stale_drops;
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
				                     "Vicon pose is %.3f s old (> max_pose_age %.3f) - stopping the "
				                     "stream. EKF2 drops EV fusion 0.4 s after the last sample.",
				                     (now_us - rx_us) * 1e-6, max_pose_age);
				return;
			}

			int64_t sample_us = stamp_source == "header"
				? rclcpp::Time(pose.header.stamp).nanoseconds() / 1000
				: rx_us;
			if (stamp_source == "header" && std::llabs(now_us - sample_us) > 1000000 && !warned_clock)
			{
				warned_clock = true;
				RCLCPP_ERROR(node->get_logger(),
				             "Vicon header stamp is %.1f s from this machine's clock. The two hosts "
				             "are not synchronised - run chrony, or pass stamp_source:=receipt and "
				             "carry the latency in EKF2_EV_DELAY.",
				             (now_us - sample_us) * 1e-6);
			}
			sample_us = std::clamp(sample_us, now_us - kMaxSampleLagUs, now_us);

			const tf2::Vector3 pIn(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
			tf2::Quaternion qIn(pose.pose.orientation.x, pose.pose.orientation.y,
			                    pose.pose.orientation.z, pose.pose.orientation.w);
			if (!std::isfinite(pIn.x()) || !std::isfinite(pIn.y()) || !std::isfinite(pIn.z()) ||
			    qIn.length2() < 1e-9 || !std::isfinite(qIn.length2()))
			{
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
				                     "Vicon pose is not finite or the quaternion is degenerate - "
				                     "dropping it.");
				return;
			}
			qIn.normalize();

			// Rotate into the workspace NED, then translate: the offset is in workspace axes.
			const tf2::Vector3 p = tf2::quatRotate(qToNed, pIn);
			tf2::Quaternion q = qToNed * qIn * qBody;
			q.normalize();
			// q and -q are the same rotation, but a bag is read by people: keep the scalar positive.
			if (q.w() < 0.0)
				q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());

			// --- frame guard ---------------------------------------------------------------
			// Checked once, then latched: an in-flight heading drift must never cut aiding out
			// from under the aircraft. Until it passes, nothing is published.
			if (!frame_validated)
			{
				if (!have_ekf_att || !yaw_aligned ||
				    now_us - yaw_aligned_since_us < static_cast<int64_t>(frame_check_settle * 1e6))
				{
					RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
					                     "Waiting for EKF2 to align its heading (cs_yaw_align) "
					                     "before trusting the Vicon frame - publishing nothing.");
					return;
				}

				// Relative rotation between what Vicon says and what EKF2 believes. Both are
				// NED/FRD, so a correct configuration leaves only EKF2's own heading bias.
				const tf2::Quaternion qErr = qEkf.inverse() * q;
				double eroll, epitch, eyaw;
				tf2::Matrix3x3(qErr).getRPY(eroll, epitch, eyaw);
				const double deg = 180.0 / M_PI;
				const double angle = std::fabs(qErr.getAngle() > M_PI ? qErr.getAngle() - 2 * M_PI
				                                                      : qErr.getAngle()) * deg;

				if (angle <= max_attitude_error)
				{
					frame_validated = true;
					RCLCPP_INFO(node->get_logger(),
					            "Frame check PASSED: Vicon and EKF2 attitudes agree to %.1f deg "
					            "(roll %.1f, pitch %.1f, yaw %.1f).",
					            angle, eroll * deg, epitch * deg, eyaw * deg);
				}
				else
				{
					// Name the knob. A world-frame error shows up as yaw alone; FLU vs FRD is a
					// 180 deg roll.
					const bool yaw_off = std::fabs(eyaw * deg) > max_attitude_error;
					const bool tilt_off = std::fabs(eroll * deg) > max_attitude_error ||
					                      std::fabs(epitch * deg) > max_attitude_error;
					const char *blame =
						yaw_off && tilt_off ? "vicon_frame AND body_frame are both wrong"
						: yaw_off           ? "vicon_frame or yaw_offset is wrong"
						: tilt_off          ? "body_frame is wrong (FLU vs FRD)"
						                    : "the Vicon subject's axes do not match the airframe";
					if (!frame_failed)
					{
						frame_failed = true;
						RCLCPP_FATAL(node->get_logger(),
						             "FRAME CHECK FAILED - publishing NOTHING. Vicon attitude is "
						             "%.1f deg from EKF2's (roll %.1f, pitch %.1f, yaw %.1f). "
						             "Most likely %s. EKF2 would have accepted this silently: a "
						             "self-consistent wrong frame re-anchors the filter and reads "
						             "as zero innovation. Fix it, or pass frame_check:=false if "
						             "you mean it.",
						             angle, eroll * deg, epitch * deg, eyaw * deg, blame);
					}
					return;
				}
			}

			VehicleOdometry odom;
			odom.timestamp = static_cast<uint64_t>(now_us);
			// EKF2 places the sample in its delay buffer by this field, not by timestamp.
			odom.timestamp_sample = static_cast<uint64_t>(sample_us);
			odom.pose_frame = VehicleOdometry::POSE_FRAME_NED;
			odom.position[0] = static_cast<float>(p.x() + position_offset[0]);
			odom.position[1] = static_cast<float>(p.y() + position_offset[1]);
			odom.position[2] = static_cast<float>(p.z() + position_offset[2]);
			// px4_msgs orders the quaternion (w,x,y,z); tf2 stores (x,y,z,w).
			odom.q[0] = static_cast<float>(q.w());
			odom.q[1] = static_cast<float>(q.x());
			odom.q[2] = static_cast<float>(q.y());
			odom.q[3] = static_cast<float>(q.z());

			// Renormalise in float: EKF2 discards the attitude unless the norm is within 1e-5 of 1,
			// and the cast to float32 is what moves it.
			const double n = std::sqrt(
				static_cast<double>(odom.q[0]) * odom.q[0] + static_cast<double>(odom.q[1]) * odom.q[1] +
				static_cast<double>(odom.q[2]) * odom.q[2] + static_cast<double>(odom.q[3]) * odom.q[3]);
			if (!std::isfinite(n) || std::fabs(1.0 - n) > kQuatNormTolerance)
				for (auto &e : odom.q)
					e = static_cast<float>(e / n);

			// Vicon supplies pose only. NaN is how VehicleOdometry says "not available"; a
			// differentiated mocap velocity would be noise EKF2 cannot gate.
			const float nan = std::numeric_limits<float>::quiet_NaN();
			const bool vel_arm = !velocity_topic.empty();
			if (vel_arm && (!have_vel || now_us - vel_rx_us > static_cast<int64_t>(max_pose_age * 1e6)))
			{
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
				                     "velocity arm: no fresh sample on %s - publishing nothing.",
				                     velocity_topic.c_str());
				return;
			}
			odom.velocity_frame = vel_arm ? VehicleOdometry::VELOCITY_FRAME_FRD
			                              : VehicleOdometry::VELOCITY_FRAME_UNKNOWN;
			for (int i = 0; i < 3; ++i)
			{
				odom.velocity[i] = vel_arm ? ev_vel[i] : nan;
				odom.angular_velocity[i] = nan;
				odom.velocity_variance[i] =
					vel_arm ? static_cast<float>(velocity_sd * velocity_sd) : nan;
				// Position is withheld on the velocity arm: EKF2 must have NO position reference,
				// or the arm answers a question it was not asked.
				odom.position[i] = vel_arm ? nan : odom.position[i];
				odom.position_variance[i] =
					vel_arm ? nan : static_cast<float>(position_sd * position_sd);
				odom.orientation_variance[i] = static_cast<float>(orientation_sd * orientation_sd);
			}
			odom.reset_counter = reset_counter;
			odom.quality = 100;

			odomPub->publish(odom);

			const int64_t age = now_us - sample_us;
			age_sum += age;
			age_max = std::max(age_max, age);
			++age_n;
			if (report_us == 0)
				report_us = now_us;
			else if (now_us - report_us >= 5000000)
			{
				RCLCPP_INFO(node->get_logger(),
				            "EV stream: %ld samples, age mean %.1f ms max %.1f ms, %ld stale drops. "
				            "Set EKF2_EV_DELAY from the mean.",
				            static_cast<long>(age_n), age_sum * 1e-3 / std::max<int64_t>(age_n, 1),
				            age_max * 1e-3, static_cast<long>(stale_drops));
				age_sum = age_max = age_n = stale_drops = 0;
				report_us = now_us;
			}
		});

	RCLCPP_INFO(node->get_logger(),
	            "Vicon bridge: %s -> %s at %.0f Hz. policy %s, frames %s/%s, yaw_offset %.4f rad, "
	            "stamp %s.",
	            vicon_topic.c_str(),
	            px4Topic<VehicleOdometry>("/fmu/in/vehicle_visual_odometry").c_str(), rate,
	            aiding_policy.c_str(), vicon_frame.c_str(), body_frame.c_str(), yaw_offset,
	            stamp_source.c_str());

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

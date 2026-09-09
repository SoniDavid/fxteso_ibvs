// Exits 0 once PX4 has the aircraft settled at the servoing altitude: started during the climb,
// the observer books it as disturbance and pos_ctrl then flies that error.
#include <rclcpp/rclcpp.hpp>
#include "quad_px4/px4_topic.hpp"
#include <px4_msgs/msg/estimator_status_flags.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <thread>

using px4_msgs::msg::EstimatorStatusFlags;
using px4_msgs::msg::VehicleLocalPosition;

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("px4_takeoff_gate");

	// Matches MIS_TAKEOFF_ALT in the airframe, which matches zD in aibvs_pos_ctrl.cpp.
	const double altitude = node->declare_parameter<double>("altitude", 2.5);
	const double tolerance = node->declare_parameter<double>("tolerance", 0.5);
	// Climbing through the altitude is not the same as settling at it.
	const double max_climb_rate = node->declare_parameter<double>("max_climb_rate", 0.3);
	const int need = node->declare_parameter<int>("required_frames", 25);
	// Wall clock, deliberately: a sim-time timeout would never fire if Gazebo failed to
	// start and /clock never advanced - exactly the case this needs to report.
	const double timeout_s = node->declare_parameter<double>("timeout", 180.0);
	// Aiding that never starts never will, so abandon rather than wait out the timeout. Zero
	// disables. Measured from EKF2's FIRST message, not this node's start, which precedes boot.
	const double aiding_deadline =
		node->declare_parameter<double>("aiding_deadline", 25.0);
	// Indoors cs_gnss_pos never comes true; yaw alignment is still required, horizontal aiding
	// is not.
	const bool require_gnss = node->declare_parameter<bool>("require_gnss", true);
	// Backstop for the other failure: PX4 never comes up at all, so no EKF2 message ever
	// arrives and the deadline above never starts counting.
	const double ekf_silence_deadline =
		node->declare_parameter<double>("ekf_silence_deadline", 60.0);

	double alt = 0.0;
	double climb = 0.0;
	bool valid = false;
	int run = 0;
	bool yaw_align = false;
	bool gnss_pos = false;
	bool ekf_seen = false;
	std::chrono::steady_clock::time_point ekf_first{};

	const rclcpp::QoS px4Qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
	// px4Topic appends the _vN suffix from px4_msgs, as every sibling node does.
	auto sub = node->create_subscription<VehicleLocalPosition>(
		quad_px4::px4Topic<VehicleLocalPosition>("/fmu/out/vehicle_local_position"), px4Qos,
		[&](const VehicleLocalPosition::ConstSharedPtr p)
		{
			valid = p->z_valid && p->v_z_valid;
			alt = -p->z;               // NED: down is positive
			climb = std::abs(p->vz);
		});

	auto ekf_sub = node->create_subscription<EstimatorStatusFlags>(
		quad_px4::px4Topic<EstimatorStatusFlags>("/fmu/out/estimator_status_flags"), px4Qos,
		[&](const EstimatorStatusFlags::ConstSharedPtr f)
		{
			if (!ekf_seen)
			{
				ekf_seen = true;
				ekf_first = std::chrono::steady_clock::now();
			}
			yaw_align = f->cs_yaw_align;
			gnss_pos = f->cs_gnss_pos;
		});

	RCLCPP_INFO(node->get_logger(),
	            "Holding the estimators until PX4 has the aircraft settled at %.2f m", altitude);

	const auto started = std::chrono::steady_clock::now();
	auto last_report = started;

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);

		const bool at_altitude =
			valid && std::abs(alt - altitude) <= tolerance && climb <= max_climb_rate;
		run = at_altitude ? run + 1 : 0;

		if (run >= need)
		{
			RCLCPP_INFO(node->get_logger(),
			            "Settled at %.2f m (climb %.2f m/s) - starting the estimators",
			            alt, climb);
			rclcpp::shutdown();
			return 0;
		}

		const auto now = std::chrono::steady_clock::now();
		const double waited = std::chrono::duration<double>(now - started).count();

		const double since_ekf =
			ekf_seen ? std::chrono::duration<double>(now - ekf_first).count() : 0.0;

		// z_valid, not just the aiding flags: GPS-denied the barometer is the only height
		// source, and it intermittently never reaches PX4 at all.
		const bool aiding_up = yaw_align && (gnss_pos || !require_gnss) && valid;

		if (aiding_deadline > 0.0 && ekf_seen && since_ekf > aiding_deadline && !aiding_up)
		{
			RCLCPP_ERROR(node->get_logger(),
			             "EKF2 never started aiding %.1f s after it began publishing: "
			             "yaw_align=%s gnss_pos=%s (gnss %s) z_valid=%s. It will not recover, "
			             "and arming stays blocked, so this run is abandoned rather than "
			             "waiting out the %.0f s timeout. Relaunch. Two known causes: a sensor "
			             "that stopped publishing has lost its noise and PX4's DataValidator "
			             "has flagged it STALE (a missing <stddev> in the model); or the "
			             "barometer never reached PX4 at all, which is intermittent and is "
			             "fatal only when there is no GNSS height to mask it.",
			             since_ekf, yaw_align ? "yes" : "no", gnss_pos ? "yes" : "no",
			             require_gnss ? "required" : "not required", valid ? "yes" : "no",
			             timeout_s);
			rclcpp::shutdown();
			return 1;
		}

		if (ekf_silence_deadline > 0.0 && !ekf_seen && waited > ekf_silence_deadline)
		{
			RCLCPP_ERROR(node->get_logger(),
			             "No estimator_status_flags after %.0f s - PX4 never came up. Check "
			             "that px4_sitl started and that the uXRCE-DDS agent is running.",
			             waited);
			rclcpp::shutdown();
			return 1;
		}

		if (waited > timeout_s)
		{
			RCLCPP_ERROR(node->get_logger(),
			             "Gave up after %.0f s: z_valid=%s altitude=%.2f/%.2f climb=%.2f. "
			             "The estimators will NOT be started. Check that PX4 armed and that "
			             "px4_offboard_bridge reported Takeoff -> Streaming.",
			             timeout_s, valid ? "yes" : "no", alt, altitude, climb);
			rclcpp::shutdown();
			return 1;
		}

		if (std::chrono::duration<double>(now - last_report).count() >= 5.0)
		{
			last_report = now;
			RCLCPP_WARN(node->get_logger(),
			            "Still waiting (%.0f s, ekf %.0f s): z_valid=%s altitude=%.2f/%.2f "
			            "climb=%.2f run=%d/%d yaw_align=%s gnss=%s%s",
			            waited, since_ekf, valid ? "yes" : "no", alt, altitude, climb, run,
			            need, yaw_align ? "yes" : "no", gnss_pos ? "yes" : "no",
			            require_gnss ? "" : " (not required)");
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	rclcpp::shutdown();
	return 1;
}

// Exits 0 once PX4 has the aircraft settled at the servoing altitude: started during the climb,
// the observer books it as disturbance and pos_ctrl then flies that error.
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <chrono>
#include <cmath>
#include <thread>

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

	double alt = 0.0;
	double climb = 0.0;
	bool valid = false;
	int run = 0;

	const rclcpp::QoS px4Qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
	auto sub = node->create_subscription<VehicleLocalPosition>(
		"/fmu/out/vehicle_local_position_v1", px4Qos,
		[&](const VehicleLocalPosition::ConstSharedPtr p)
		{
			valid = p->z_valid && p->v_z_valid;
			alt = -p->z;               // NED: down is positive
			climb = std::abs(p->vz);
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
			            "Still waiting (%.0f s): z_valid=%s altitude=%.2f/%.2f climb=%.2f run=%d/%d",
			            waited, valid ? "yes" : "no", alt, altitude, climb, run, need);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	rclcpp::shutdown();
	return 1;
}

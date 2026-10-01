// A stand-in Vicon receiver for SITL: Gazebo ground truth published as the PoseStamped the real
// receiver produces, so vicon_px4_bridge under test is the one that flies. Simulation only -
// quad_gz_sim is not built on the Pi, so this can never ship on the aircraft.
//
// The gz world is ENU, which is also how a Z-up Vicon volume is calibrated, so the pose is
// published unrotated and the bridge runs its vicon_frame:=enu path exactly as it will in the lab.
// Three frames are in play and only the middle one is PX4's:
//   gz world   ENU                      what /quad_state carries, body FLU
//   PX4 NED    (y, x, -z) of gz         what EKF2 believes; the bridge must feed THIS
//   workspace  (x, -y, -z) of gz        = Rz(-pi/2) * PX4 NED, applied by px4_state_adapter
// An earlier version rotated by +pi/2 here on the belief that gz was NWU. That landed the stream
// in the workspace frame - 90 deg out - and because a self-consistent wrong frame produces near
// zero innovations, EKF2 accepted it silently. See vicon_px4_bridge's frame guard.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <random>
#include <string>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("vicon_sim");

	const std::string state_topic =
		node->declare_parameter<std::string>("state_topic", "quad_state");
	const std::string vicon_topic =
		node->declare_parameter<std::string>("vicon_topic", "/vicon/quad/quad");
	// A real Vicon system streams faster than the bridge resamples it.
	const double rate = node->declare_parameter<double>("rate", 100.0);

	// Defaults are deliberately non-zero: a bridge only ever tested on a clean feed proves nothing
	// about the one that meets a lab.
	const double position_noise_sd = node->declare_parameter<double>("position_noise_sd", 0.001);
	const double yaw_noise_sd = node->declare_parameter<double>("yaw_noise_sd", 0.002);
	// End-to-end latency to imitate: capture, DataStream, WiFi. Set EKF2_EV_DELAY against it.
	const double latency = node->declare_parameter<double>("latency", 0.02);
	// Occlusion. Per-tick probability of entering a dropout, and how long one lasts.
	const double dropout_probability = node->declare_parameter<double>("dropout_probability", 0.0);
	const double dropout_duration = node->declare_parameter<double>("dropout_duration", 0.5);

	// The laptop receiver stamps on its own wall clock, and the bridge reads it on the system
	// clock too. Using it here keeps stamp_source:=header on the same path it takes on hardware.
	rclcpp::Clock sys_clock(RCL_SYSTEM_TIME);

	auto posePub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
		vicon_topic, rclcpp::QoS(10));

	std::deque<std::pair<int64_t, geometry_msgs::msg::PoseStamped>> delay_line;
	bool have_state = false;
	int64_t dropout_until = 0;

	std::mt19937 rng(std::random_device{}());
	std::normal_distribution<double> pos_noise(0.0, position_noise_sd);
	std::normal_distribution<double> yaw_noise(0.0, yaw_noise_sd);
	std::uniform_real_distribution<double> unit(0.0, 1.0);

	auto stateSub = node->create_subscription<nav_msgs::msg::Odometry>(
		state_topic, rclcpp::QoS(100),
		[&](const nav_msgs::msg::Odometry::ConstSharedPtr odom)
		{
			// The gz world is already ENU: pass the pose through, adding only the imitated noise.
			const auto &p = odom->pose.pose.position;

			tf2::Quaternion qGz;
			tf2::fromMsg(odom->pose.pose.orientation, qGz);
			tf2::Quaternion q = qGz * tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_noise(rng));
			q.normalize();

			geometry_msgs::msg::PoseStamped out;
			out.header.frame_id = "world";
			out.header.stamp = sys_clock.now();
			out.pose.position.x = p.x + pos_noise(rng);
			out.pose.position.y = p.y + pos_noise(rng);
			out.pose.position.z = p.z + pos_noise(rng);
			out.pose.orientation = tf2::toMsg(q);

			delay_line.emplace_back(sys_clock.now().nanoseconds() / 1000, out);
			have_state = true;
		});

	auto timer = node->create_wall_timer(
		std::chrono::duration<double>(1.0 / rate),
		[&]()
		{
			const int64_t now_us = sys_clock.now().nanoseconds() / 1000;

			if (now_us < dropout_until)
			{
				delay_line.clear();
				return;
			}
			if (dropout_probability > 0.0 && unit(rng) < dropout_probability)
			{
				dropout_until = now_us + static_cast<int64_t>(dropout_duration * 1e6);
				RCLCPP_WARN(node->get_logger(), "Simulated Vicon dropout for %.2f s.",
				            dropout_duration);
				return;
			}

			// Release the newest sample that is at least `latency` old.
			const int64_t due = now_us - static_cast<int64_t>(latency * 1e6);
			geometry_msgs::msg::PoseStamped out;
			bool ready = false;
			while (!delay_line.empty() && delay_line.front().first <= due)
			{
				out = delay_line.front().second;
				delay_line.pop_front();
				ready = true;
			}
			if (ready)
				posePub->publish(out);
			else if (!have_state)
				RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000,
				                     "No %s yet - publishing no pose.", state_topic.c_str());
		});

	RCLCPP_INFO(node->get_logger(),
	            "vicon_sim: %s (gz ENU, unrotated) -> %s at %.0f Hz, latency %.0f ms, noise %.1f mm, "
	            "dropout p=%.4f.",
	            state_topic.c_str(), vicon_topic.c_str(), rate, latency * 1e3,
	            position_noise_sd * 1e3, dropout_probability);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

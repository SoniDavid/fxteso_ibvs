// Exits 0 once closed-loop servoing is actually possible: quad placed, target placed, markers
// decoded, and the estimator chain live. sim.launch.py starts pos_ctrl / att_ctrl on that exit.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <chrono>
#include <cstdio>

bool have_quad = false;
bool have_tgt = false;
int lock_run = 0;

// image_features assigns exactly this when it has fewer than 4 markers - literals, never
// arithmetic, so exact comparison is intended and not a float-equality mistake.
static bool isNoLockSentinel(const geometry_msgs::msg::Quaternion &f)
{
	return f.x == 0.0 && f.y == 0.0 && f.z == 1.0 && f.w == 0.0;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("ibvs_gate");

	// Consecutive good frames, so a single lucky detection during start-up does not arm
	// the controllers on a lock that is about to be lost again.
	const int need = node->declare_parameter<int>("required_lock_frames", 10);
	// Wall clock, deliberately: a sim-time timeout would never fire if Gazebo failed to
	// start and /clock never advanced - exactly the case this needs to report.
	const double timeout_s = node->declare_parameter<double>("timeout", 120.0);

	// td_linear holds 5.0 s of sim time, fixed_eso 4.7, td_attitude 4.5, and all of them
	// publish placeholder values before that - so a message is not proof they are live.
	// Under plant:=analytic the plant's own 5 s hold made this implicit; under plant:=gazebo
	// physics runs from t=0 and markers lock at ~1.9 s.
	const double estimators_ready =
		node->declare_parameter<double>("estimators_ready", 5.0);

	auto quad_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 10, [](const geometry_msgs::msg::Vector3::ConstSharedPtr) { have_quad = true; });
	auto tgt_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 10, [](const geometry_msgs::msg::Vector3::ConstSharedPtr) { have_tgt = true; });
	auto feat_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"ImFeat_vector", 10,
		[](const geometry_msgs::msg::Quaternion::ConstSharedPtr f)
		{
			lock_run = isNoLockSentinel(*f) ? 0 : lock_run + 1;
		});

	RCLCPP_INFO(node->get_logger(),
	            "Holding the controllers until the plant, the target and a %d-frame marker "
	            "lock are all up", need);

	const auto started = std::chrono::steady_clock::now();
	auto last_report = started;

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);

		const double sim_t = node->now().seconds();

		if (have_quad && have_tgt && lock_run >= need && sim_t >= estimators_ready)
		{
			RCLCPP_INFO(node->get_logger(),
			            "Quad and target placed, markers locked for %d frames, estimators up "
			            "at sim t=%.3f - starting the controllers",
			            lock_run, sim_t);
			rclcpp::shutdown();
			return 0;
		}

		const auto now = std::chrono::steady_clock::now();
		const double waited = std::chrono::duration<double>(now - started).count();

		if (waited > timeout_s)
		{
			RCLCPP_ERROR(node->get_logger(),
			             "Gave up after %.0f s: quad_position=%s tgt_position=%s "
			             "marker_lock=%d/%d sim_t=%.2f/%.2f. The controllers will NOT be started, because "
			             "they would servo on the no-lock feature vector. Check that the "
			             "camera is rendering (/quad/camera/image_raw) and that the quad "
			             "starts above the target.",
			             timeout_s, have_quad ? "yes" : "no", have_tgt ? "yes" : "no",
			             lock_run, need, sim_t, estimators_ready);
			rclcpp::shutdown();
			return 1;
		}

		if (std::chrono::duration<double>(now - last_report).count() >= 5.0)
		{
			last_report = now;
			RCLCPP_WARN(node->get_logger(),
			            "Still waiting (%.0f s): quad_position=%s tgt_position=%s "
			            "marker_lock=%d/%d sim_t=%.2f/%.2f",
			            waited, have_quad ? "yes" : "no", have_tgt ? "yes" : "no",
			            lock_run, need, sim_t, estimators_ready);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	rclcpp::shutdown();
	return 1;
}

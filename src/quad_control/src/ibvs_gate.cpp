// Exits 0 once closed-loop servoing is actually possible: quad placed, target placed, markers
// decoded, and the estimator chain live. sim.launch.py starts pos_ctrl / att_ctrl on that exit.

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>

bool have_quad = false;
bool have_tgt = false;
int lock_run = 0;
geometry_msgs::msg::Quaternion last_feat;

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
	// A decoded lock is not a usable initial condition: handing over at an arbitrary relative
	// yaw makes it an accident. Zero disables the test.
	const double max_feature_error =
		node->declare_parameter<double>("max_feature_error", 0.15);
	// Wall clock, deliberately: a sim-time timeout would never fire if Gazebo failed to
	// start and /clock never advanced - exactly the case this needs to report.
	const double timeout_s = node->declare_parameter<double>("timeout", 120.0);

	// The estimators publish placeholders during their start-up hold, so a message is not proof
	// they are live. Measured from this node's start, not sim epoch.
	const double estimators_ready =
		node->declare_parameter<double>("estimators_ready", 5.0);

	// tgt_position is the simulator's; on hardware the target is a printed plate and no such
	// topic exists. Only its arrival is tested, never its value.
	const bool require_target = node->declare_parameter<bool>("require_target", true);

	auto quad_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 10, [](const geometry_msgs::msg::Vector3::ConstSharedPtr) { have_quad = true; });
	auto tgt_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 10, [](const geometry_msgs::msg::Vector3::ConstSharedPtr) { have_tgt = true; });
	auto feat_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"ImFeat_vector", 10,
		[&](const geometry_msgs::msg::Quaternion::ConstSharedPtr f)
		{
			const bool aligned =
				max_feature_error <= 0.0 ||
				(std::abs(f->w) <= max_feature_error &&
				 std::abs(f->x) <= max_feature_error &&
				 std::abs(f->y) <= max_feature_error);
			lock_run = (isNoLockSentinel(*f) || !aligned) ? 0 : lock_run + 1;
			last_feat = *f;
		});

	RCLCPP_INFO(node->get_logger(),
	            "Holding the controllers until the plant,%s a %d-frame marker lock are all up",
	            require_target ? " the target and" : " (target not required) and", need);

	const auto started = std::chrono::steady_clock::now();
	auto last_report = started;

	// Latched on the first non-zero sample, not here: with use_sim_time the clock reads 0 until
	// the first /clock arrives, and latching that would revert this to an absolute check.
	double sim_start = -1.0;

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);

		const double sim_now = node->now().seconds();
		if (sim_start < 0.0 && sim_now > 0.0)
			sim_start = sim_now;
		const double sim_t = sim_start < 0.0 ? 0.0 : sim_now - sim_start;

		if (have_quad && (have_tgt || !require_target) && lock_run >= need
		    && sim_t >= estimators_ready)
		{
			RCLCPP_INFO(node->get_logger(),
			            "Quad and target placed, markers locked and aligned for %d frames "
			            "(qpsi %.3f, qx %.3f, qy %.3f, limit %.3f), estimators up %.3f s "
			            "after this gate started - starting the controllers",
			            lock_run, last_feat.w, last_feat.x, last_feat.y,
			            max_feature_error, sim_t);
			rclcpp::shutdown();
			return 0;
		}

		const auto now = std::chrono::steady_clock::now();
		const double waited = std::chrono::duration<double>(now - started).count();

		if (waited > timeout_s)
		{
			RCLCPP_ERROR(node->get_logger(),
			             "Gave up after %.0f s: quad_position=%s tgt_position=%s "
			             "marker_lock=%d/%d sim_t=%.2f/%.2f feat=(%.3f,%.3f,%.3f) limit=%.3f. "
			             "The controllers will NOT be started, because "
			             "they would servo on the no-lock feature vector. Check that the "
			             "camera is rendering (/quad/camera/image_raw) and that the quad "
			             "starts above the target.",
			             timeout_s, have_quad ? "yes" : "no",
			             have_tgt ? "yes" : (require_target ? "no" : "no (not required)"),
			             lock_run, need, sim_t, estimators_ready,
			             last_feat.w, last_feat.x, last_feat.y, max_feature_error);
			rclcpp::shutdown();
			return 1;
		}

		if (std::chrono::duration<double>(now - last_report).count() >= 5.0)
		{
			last_report = now;
			RCLCPP_WARN(node->get_logger(),
			            "Still waiting (%.0f s): quad_position=%s tgt_position=%s "
			            "marker_lock=%d/%d sim_t=%.2f/%.2f feat=(%.3f,%.3f,%.3f) limit=%.3f",
			            waited, have_quad ? "yes" : "no",
			            have_tgt ? "yes" : (require_target ? "no" : "no (not required)"),
			            lock_run, need, sim_t, estimators_ready,
			            last_feat.w, last_feat.x, last_feat.y, max_feature_error);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}

	rclcpp::shutdown();
	return 1;
}

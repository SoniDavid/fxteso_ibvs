// Loop pacing on the ROS clock, so the stack runs in simulation time.
// rclcpp::Rate is wall-clock only. Logs a `loop:` rate line every ~10 s; FXTESO_LOOP_STATS=0 silences it.

#ifndef QUAD_COMMON__SIM_RATE_HPP_
#define QUAD_COMMON__SIM_RATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace fxteso
{

inline std::chrono::milliseconds pollInterval()
{
	return std::chrono::milliseconds(1);
}

inline void spinUntil(rclcpp::executors::SingleThreadedExecutor &exec,
                      const rclcpp::Node::SharedPtr &node,
                      const rclcpp::Time &target)
{
	while (rclcpp::ok() && node->now() < target)
	{
		exec.spin_some();
		std::this_thread::sleep_for(pollInterval());
	}
}

class SimRate
{
public:
	SimRate(rclcpp::Node::SharedPtr node, double hz)
	: node_(node),
	  hz_(hz),
	  period_(rclcpp::Duration::from_seconds(1.0 / hz)),
	  next_(node->now()),
	  start_(node->now())
	{
		exec_.add_node(node_);
		const char *env = std::getenv("FXTESO_LOOP_STATS");
		stats_enabled_ = (env == nullptr) || (std::string(env) != "0");
		stats_period_ = std::max<std::uint64_t>(
			1, static_cast<std::uint64_t>(std::llround(hz * 10.0)));
	}

	~SimRate()
	{
		if (iters_ > 0)
			logStats(" [final]", false);
		exec_.remove_node(node_);
	}

	void sleep()
	{
		++iters_;
		const rclcpp::Time now = node_->now();
		if (have_last_)
			max_gap_ = std::max(max_gap_, (now - last_entry_).seconds());
		last_entry_ = now;
		have_last_ = true;

		next_ = next_ + period_;
		if (next_ < now)
		{
			// Resync rather than catch up, and count it: a silent overrun is a wrong integration step.
			++overruns_;
			worst_overshoot_ = std::max(worst_overshoot_, (now - next_).seconds());
			next_ = now + period_;
		}

		if (stats_enabled_ && (iters_ % stats_period_ == 0))
		{
			logStats("", overruns_ > reported_overruns_);
			reported_overruns_ = overruns_;
		}

		spinUntil(exec_, node_, next_);
	}

	// For the fixed start-up delays the nodes use to stagger themselves. Resets the stats.
	void sleepFor(double seconds)
	{
		const rclcpp::Time target = node_->now() + rclcpp::Duration::from_seconds(seconds);
		spinUntil(exec_, node_, target);
		next_ = node_->now();
		start_ = next_;
		iters_ = 0;
		overruns_ = 0;
		reported_overruns_ = 0;
		worst_overshoot_ = 0.0;
		max_gap_ = 0.0;
		have_last_ = false;
	}

private:
	// WARN when overruns grew since the last line: the observer and controller assume the period.
	void logStats(const char *tag, bool warn)
	{
		const double elapsed = (last_entry_ - start_).seconds();
		const double achieved = elapsed > 0.0 ? iters_ / elapsed : 0.0;
		char line[256];
		std::snprintf(line, sizeof(line),
		              "loop: target %.1f Hz | achieved %.1f Hz | %lu/%lu overruns (%.1f%%) | "
		              "worst +%.1f ms | max gap %.1f ms%s",
		              hz_, achieved,
		              static_cast<unsigned long>(overruns_),
		              static_cast<unsigned long>(iters_),
		              100.0 * overruns_ / std::max<std::uint64_t>(1, iters_),
		              worst_overshoot_ * 1e3, max_gap_ * 1e3, tag);
		if (warn)
			RCLCPP_WARN(node_->get_logger(), "%s", line);
		else
			RCLCPP_INFO(node_->get_logger(), "%s", line);
	}

	rclcpp::Node::SharedPtr node_;
	rclcpp::executors::SingleThreadedExecutor exec_;
	double hz_;
	rclcpp::Duration period_;
	rclcpp::Time next_;
	rclcpp::Time start_;
	rclcpp::Time last_entry_{0, 0, RCL_ROS_TIME};

	bool stats_enabled_ = true;
	bool have_last_ = false;
	std::uint64_t stats_period_ = 1;
	std::uint64_t iters_ = 0;
	std::uint64_t overruns_ = 0;
	std::uint64_t reported_overruns_ = 0;
	double worst_overshoot_ = 0.0;
	double max_gap_ = 0.0;
};

}  // namespace fxteso

#endif  // QUAD_COMMON__SIM_RATE_HPP_

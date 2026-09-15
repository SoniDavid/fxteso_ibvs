// Loop pacing on the ROS clock, so the stack runs in simulation time.
// rclcpp::Rate is wall-clock only.

#ifndef QUAD_COMMON__SIM_RATE_HPP_
#define QUAD_COMMON__SIM_RATE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
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
	  period_(rclcpp::Duration::from_seconds(1.0 / hz)),
	  next_(node->now())
	{
		exec_.add_node(node_);
	}

	~SimRate()
	{
		exec_.remove_node(node_);
	}

	void sleep()
	{
		next_ = next_ + period_;
		const rclcpp::Time now = node_->now();
		if (next_ < now)
		{
			// Resync rather than catch up, and report it: the observer and controller
			// integrate at a fixed assumed step, so a silent overrun is a wrong step.
			++overruns_;
			const double late = (now - next_).seconds() + period_.seconds();
			worst_late_ = std::max(worst_late_, late);
			next_ = now + period_;
		}
		spinUntil(exec_, node_, next_);

		// Throttled on the loop's own clock.
		if (overruns_ > 0 && (now - last_report_).seconds() >= 10.0)
		{
			RCLCPP_WARN(node_->get_logger(),
			            "loop overran its %.1f ms period %ld times in the last %.0f s "
			            "(worst %.1f ms). The fixed integration step assumes the period.",
			            period_.seconds() * 1000.0, overruns_,
			            (now - last_report_).seconds(), worst_late_ * 1000.0);
			overruns_ = 0;
			worst_late_ = 0.0;
			last_report_ = now;
		}
		else if (overruns_ == 0 && (now - last_report_).seconds() >= 10.0)
			last_report_ = now;
	}

	// For the fixed start-up delays the nodes use to stagger themselves.
	void sleepFor(double seconds)
	{
		const rclcpp::Time target = node_->now() + rclcpp::Duration::from_seconds(seconds);
		spinUntil(exec_, node_, target);
		next_ = node_->now();
	}

private:
	rclcpp::Node::SharedPtr node_;
	rclcpp::executors::SingleThreadedExecutor exec_;
	rclcpp::Duration period_;
	rclcpp::Time next_;
	long overruns_ = 0;
	double worst_late_ = 0.0;
	rclcpp::Time last_report_ = node_->now();
};

}  // namespace fxteso

#endif  // QUAD_COMMON__SIM_RATE_HPP_

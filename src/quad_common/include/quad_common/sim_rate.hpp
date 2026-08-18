// Loop pacing on the ROS clock, so the stack runs in simulation time.
// rclcpp::Rate is wall-clock only.

#ifndef QUAD_COMMON__SIM_RATE_HPP_
#define QUAD_COMMON__SIM_RATE_HPP_

#include <rclcpp/rclcpp.hpp>

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
			next_ = now + period_;   // fell a period behind; resync instead of catching up
		spinUntil(exec_, node_, next_);
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
};

}  // namespace fxteso

#endif  // QUAD_COMMON__SIM_RATE_HPP_

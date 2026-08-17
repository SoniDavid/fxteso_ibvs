// Feeds the control stack from Gazebo physics instead of uav_dynamics.cpp, publishing the same
// five topics. Exact inverse of gz_pose_broadcaster.cpp. 
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// gz world -> ROS NED, and back: the involution is its own inverse.
static geometry_msgs::msg::Vector3 flip(double x, double y, double z)
{
	geometry_msgs::msg::Vector3 v;
	v.x = x;
	v.y = -y;
	v.z = -z;
	return v;
}

// The control stack needs unbounded angles; getRPY() returns (-pi, pi]. A step above pi at
// 100 Hz is always a wrap, never motion. See docs/angle-wrapping.md.
class Unwrapped
{
public:
	double operator()(double raw)
	{
		if (this->started)
		{
			const double d = raw - this->prev;
			if (d > M_PI)
				this->turns -= 2.0 * M_PI;
			else if (d < -M_PI)
				this->turns += 2.0 * M_PI;
		}
		this->started = true;
		this->prev = raw;
		return raw + this->turns;
	}

private:
	bool started{false};
	double prev{0.0};
	double turns{0.0};
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("gz_state_adapter");

	auto positionPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_position", 100);
	auto attitudePub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude", 100);
	auto velocityPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity", 100);
	auto attVelPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude_velocity", 100);
	auto velBFPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity_BF", 100);

	// /disturbances plus whatever WindEffects applied - the observer's ground truth.
	auto distTotalPub = node->create_publisher<geometry_msgs::msg::Vector3>("disturbances_total", 100);
	auto extSub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"external_force", 100,
		[&](const geometry_msgs::msg::Vector3::ConstSharedPtr f)
		{
			// The plant applies "-dist"; invert to match how /disturbances is reported.
			distTotalPub->publish(flip(-f->x, -f->y, -f->z));
		});

	Unwrapped unwrapRoll, unwrapPitch, unwrapYaw;

	// Debug only: false feeds a wrapped attitude, as a quaternion source would.
	const bool unwrap = node->declare_parameter<bool>("unwrap_attitude", true);

	// From the BodyWrench plugin: the ECM's own values, not a numerical derivative.
	auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
		"quad_state", 100,
		[&](const nav_msgs::msg::Odometry::ConstSharedPtr odom)
		{
			const auto &p = odom->pose.pose.position;
			positionPub->publish(flip(p.x, p.y, p.z));

			tf2::Quaternion q;
			tf2::fromMsg(odom->pose.pose.orientation, q);
			double roll, pitch, yaw;
			tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
			attitudePub->publish(unwrap
				? flip(unwrapRoll(roll), unwrapPitch(pitch), unwrapYaw(yaw))
				: flip(roll, pitch, yaw));

			// twist is in the body frame (nav_msgs convention), so this is the plant's
			// linear_velocity_BF and its body angular velocity.
			const auto &vb = odom->twist.twist.linear;
			const auto &w = odom->twist.twist.angular;
			velBFPub->publish(flip(vb.x, vb.y, vb.z));
			attVelPub->publish(flip(w.x, w.y, w.z));

			// Inertial velocity, rotated out of the body frame by the true attitude rather
			// than by the plant's small-angle rotation matrix.
			const tf2::Vector3 vWorld = tf2::quatRotate(q, tf2::Vector3(vb.x, vb.y, vb.z));
			velocityPub->publish(flip(vWorld.x(), vWorld.y(), vWorld.z()));
		});

	RCLCPP_INFO(node->get_logger(), "Gazebo physics plant: /quad_state -> quad_position/attitude/velocity");

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

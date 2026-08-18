// Feeds the control stack from PX4's EKF2, publishing the same five topics as uav_dynamics
// and gz_state_adapter so nothing downstream knows which plant ran.
//
// PX4 is already NED/FRD, so no handedness flip - but its North is geographic while this
// workspace calls Gazebo +x North, hence frame_yaw_offset.
#include <rclcpp/rclcpp.hpp>
#include "quad_common/unwrapped.hpp"
#include "quad_px4/px4_topic.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cmath>

static geometry_msgs::msg::Vector3 vec3(double x, double y, double z)
{
	geometry_msgs::msg::Vector3 v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("px4_state_adapter");

	auto positionPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_position", 100);
	auto attitudePub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude", 100);
	auto velocityPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity", 100);
	auto attVelPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude_velocity", 100);
	auto velBFPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity_BF", 100);

	fxteso::Unwrapped unwrapRoll, unwrapPitch, unwrapYaw;
	uint8_t last_reset = 0;

	// Debug only: false feeds a wrapped attitude, matching gz_state_adapter's switch.
	const bool unwrap = node->declare_parameter<bool>("unwrap_attitude", true);

	// EKF2 anchors its local frame where the aircraft initialised; the other plants report against
	// the Gazebo world origin, which is also what tgt_position and the TF tree use. Set these to
	// the spawn pose from worlds/ibvs.sdf in NED.
	const double origin_n = node->declare_parameter<double>("origin_north", 0.0);
	const double origin_e = node->declare_parameter<double>("origin_east", 0.0);
	const double origin_d = node->declare_parameter<double>("origin_down", 0.0);

	// Rotation taking EKF2's NED into the workspace's, about the down axis. -pi/2 for the ENU
	// world. Applied to position, both velocities and attitude, so the state stays self-consistent.
	const double frame_yaw = node->declare_parameter<double>("frame_yaw_offset", 0.0);
	const tf2::Quaternion qFrame(tf2::Vector3(0, 0, 1), frame_yaw);
	const tf2::Matrix3x3 rFrame(qFrame);

	// uXRCE-DDS publishes best-effort; a reliable subscription silently receives nothing.
	const rclcpp::QoS px4Qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

	// One message carries the whole state: position and velocity in NED, attitude as a
	// quaternion, and body-frame angular velocity. Rate-limited to 100 Hz by PX4.
	using px4_msgs::msg::VehicleOdometry;
	auto sub = node->create_subscription<VehicleOdometry>(
		quad_px4::px4Topic<VehicleOdometry>("/fmu/out/vehicle_odometry"), px4Qos,
		[&](const VehicleOdometry::ConstSharedPtr odom)
		{
			// PX4 marks an unavailable field with NaN rather than omitting the message.
			if (!std::isfinite(odom->position[0]) || !std::isfinite(odom->q[0]))
				return;

			// Rotate into the workspace frame first, then translate: the offset is expressed in workspace
			// axes.
			const tf2::Vector3 pWs = rFrame * tf2::Vector3(
				odom->position[0], odom->position[1], odom->position[2]);
			positionPub->publish(vec3(pWs.x() + origin_n,
			                          pWs.y() + origin_e,
			                          pWs.z() + origin_d));

			// px4_msgs quaternions are (w,x,y,z); tf2 takes (x,y,z,w). Pre-multiplying by qFrame shifts
			// yaw onto the workspace datum and leaves roll and pitch alone.
			const tf2::Quaternion q(odom->q[1], odom->q[2], odom->q[3], odom->q[0]);
			const tf2::Quaternion qWs = qFrame * q;
			double roll, pitch, yaw;
			tf2::Matrix3x3(qWs).getRPY(roll, pitch, yaw);

			// EKF2 corrects its heading in steps while converging, and a step over pi is indistinguishable
			// from a wrap. reset_counter is PX4 telling us which jumps are resets rather than motion.
			if (odom->reset_counter != last_reset)
			{
				last_reset = odom->reset_counter;
				unwrapRoll.reseed(roll);
				unwrapPitch.reseed(pitch);
				unwrapYaw.reseed(yaw);
			}

			attitudePub->publish(unwrap
				? vec3(unwrapRoll(roll), unwrapPitch(pitch), unwrapYaw(yaw))
				: vec3(roll, pitch, yaw));

			// Body-frame already, so the datum rotation does not touch it.
			attVelPub->publish(vec3(odom->angular_velocity[0], odom->angular_velocity[1],
			                        odom->angular_velocity[2]));

			// EKF2 reports velocity in NED; the stack wants both that and its body-frame
			// counterpart, so rotate rather than differentiating anything.
			const tf2::Vector3 vWs = rFrame * tf2::Vector3(
				odom->velocity[0], odom->velocity[1], odom->velocity[2]);
			velocityPub->publish(vec3(vWs.x(), vWs.y(), vWs.z()));

			const tf2::Vector3 vBody = tf2::quatRotate(qWs.inverse(), vWs);
			velBFPub->publish(vec3(vBody.x(), vBody.y(), vBody.z()));
		});

	RCLCPP_INFO(node->get_logger(),
	            "PX4 plant: /fmu/out/vehicle_odometry -> quad_position/attitude/velocity");

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}

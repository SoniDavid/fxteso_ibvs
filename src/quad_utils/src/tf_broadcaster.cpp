// TF, trajectory trails and markers for Foxglove. Pure consumer; nothing in the control loop
// reads it. The ROS -> ENU mapping is copied from gz_pose_broadcaster.cpp, not re-derived.

#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <string>

// nav_msgs/Path resends the whole trail every message, so bandwidth grows with the square
// of the run. 5 Hz with a 1000-pose cap keeps a 200 s trail at ~0.4 MB/s.
static const size_t PATH_LEN = 1000;
static const int PATH_RATE_DIV = 10;
static const char *WORLD_FRAME = "world";

float quad_x = 0, quad_y = 0, quad_z = 0;
float roll = 0, pitch = 0, yaw = 0;

float tgt_x = 0, tgt_y = 0, tgt_z = 0;
float tgt_yaw = 0;

void quadPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr p)
{
	quad_x = p->x;
	quad_y = p->y;
	quad_z = p->z;
}

void quadAttCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr a)
{
	roll = a->x;
	pitch = a->y;
	yaw = a->z;
}

void tgtPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr p)
{
	tgt_x = p->x;
	tgt_y = p->y;
	tgt_z = p->z;
}

void tgtYawCallback(const std_msgs::msg::Float64::ConstSharedPtr y)
{
	tgt_yaw = y->data;
}

static geometry_msgs::msg::TransformStamped makeTf(const rclcpp::Time &stamp,
                                                   const std::string &child,
                                                   double x, double y, double z,
                                                   const tf2::Quaternion &q)
{
	geometry_msgs::msg::TransformStamped t;
	t.header.stamp = stamp;
	t.header.frame_id = WORLD_FRAME;
	t.child_frame_id = child;
	t.transform.translation.x = x;
	t.transform.translation.y = y;
	t.transform.translation.z = z;
	t.transform.rotation.x = q.x();
	t.transform.rotation.y = q.y();
	t.transform.rotation.z = q.z();
	t.transform.rotation.w = q.w();
	return t;
}

static void appendPose(nav_msgs::msg::Path &path, const rclcpp::Time &stamp,
                       double x, double y, double z)
{
	geometry_msgs::msg::PoseStamped ps;
	ps.header.stamp = stamp;
	ps.header.frame_id = WORLD_FRAME;
	ps.pose.position.x = x;
	ps.pose.position.y = y;
	ps.pose.position.z = z;
	ps.pose.orientation.w = 1.0;

	path.header.stamp = stamp;
	path.header.frame_id = WORLD_FRAME;
	path.poses.push_back(ps);
	if (path.poses.size() > PATH_LEN)
		path.poses.erase(path.poses.begin());
}

// Primitives, not the F450 STL: Foxglove desktop will not resolve package:// mesh URIs.
static visualization_msgs::msg::Marker makeBox(const std::string &frame, int id,
                                               double sx, double sy, double sz,
                                               float r, float g, float b)
{
	visualization_msgs::msg::Marker m;
	m.header.frame_id = frame;
	m.ns = "ibvs";
	m.id = id;
	m.type = visualization_msgs::msg::Marker::CUBE;
	m.action = visualization_msgs::msg::Marker::ADD;
	m.pose.orientation.w = 1.0;
	m.scale.x = sx;
	m.scale.y = sy;
	m.scale.z = sz;
	m.color.r = r;
	m.color.g = g;
	m.color.b = b;
	m.color.a = 1.0;
	return m;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("tf_broadcaster");

	fxteso::SimRate loop_rate(node, 50);

	auto quad_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 1, quadPosCallback);
	auto quad_att_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_attitude", 1, quadAttCallback);
	auto tgt_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 1, tgtPosCallback);
	auto tgt_yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
		"tgt_yaw", 1, tgtYawCallback);

	auto quad_path_pub = node->create_publisher<nav_msgs::msg::Path>("quad_path", 1);
	auto tgt_path_pub = node->create_publisher<nav_msgs::msg::Path>("tgt_path", 1);
	auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
		"ibvs_markers", 1);

	tf2_ros::TransformBroadcaster tf_bc(node);

	nav_msgs::msg::Path quad_path, tgt_path;

	tf2::Quaternion q_rot;
	q_rot.setRPY(3.141592, 0, 0);

	// Built once, republished at 1 Hz for panels that connect late.
	visualization_msgs::msg::MarkerArray markers;
	markers.markers.push_back(makeBox("target", 0, 0.90, 0.75, 0.01, 0.9f, 0.9f, 0.9f));
	markers.markers.push_back(makeBox("quad", 1, 0.35, 0.35, 0.08, 0.2f, 0.5f, 0.9f));

	int tick = 0;

	while (rclcpp::ok())
	{
		const rclcpp::Time stamp = node->now();

		tf2::Quaternion q_att;
		q_att.setRPY(roll, pitch, yaw);
		tf2::Quaternion q_quad = q_rot * q_att;
		q_quad.normalize();

		tf2::Quaternion q_tgt;
		q_tgt.setRPY(0, 0, -tgt_yaw);
		q_tgt.normalize();

		tf_bc.sendTransform(makeTf(stamp, "quad", quad_x, -quad_y, -quad_z, q_quad));
		tf_bc.sendTransform(makeTf(stamp, "target", tgt_x, -tgt_y, -tgt_z, q_tgt));

		if (tick % PATH_RATE_DIV == 0)
		{
			appendPose(quad_path, stamp, quad_x, -quad_y, -quad_z);
			appendPose(tgt_path, stamp, tgt_x, -tgt_y, -tgt_z);
			quad_path_pub->publish(quad_path);
			tgt_path_pub->publish(tgt_path);
		}

		if (tick % 50 == 0)
		{
			for (auto &m : markers.markers)
				m.header.stamp = stamp;
			marker_pub->publish(markers);
		}
		tick++;

		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}

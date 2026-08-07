// Teleports the F450 and the ArUco target in Gazebo to follow the poses integrated by
// uav_dynamics / target_position on the ROS side. See documentation.md.
//
// NOTE: in gz-sim 8 set_pose_vector is a SERVICE, not a topic. Publishing to it as a topic
// silently succeeds and does nothing. Requests are async, so the 100 Hz loop never blocks.
//
// The frame conventions below encode the ROS(NED-ish) -> Gazebo(ENU) mapping the whole
// stack is calibrated against; the quad's pi roll is what points the camera down.

#include <rclcpp/rclcpp.hpp>
#include "fxteso_ibvs/sim_rate.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <cmath>
#include <string>

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

// Fire-and-forget: the reply only says whether Gazebo accepted the command, and there is
// nothing useful to do at 100 Hz if it did not.
static void onSetPose(const gz::msgs::Boolean &, const bool)
{
}

static void fill(gz::msgs::Pose *pose, const std::string &name,
                 double x, double y, double z, const tf2::Quaternion &q)
{
	pose->set_name(name);
	pose->mutable_position()->set_x(x);
	pose->mutable_position()->set_y(y);
	pose->mutable_position()->set_z(z);
	pose->mutable_orientation()->set_x(q.x());
	pose->mutable_orientation()->set_y(q.y());
	pose->mutable_orientation()->set_z(q.z());
	pose->mutable_orientation()->set_w(q.w());
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("gz_pose_broadcaster");

	const std::string world = node->declare_parameter<std::string>("world", "ibvs");
	const std::string service = "/world/" + world + "/set_pose_vector";

	fxteso::SimRate loop_rate(node, 100);

	auto quad_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 100, quadPosCallback);
	auto quad_att_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_attitude", 100, quadAttCallback);
	auto tgt_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 100, tgtPosCallback);
	auto tgt_yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
		"tgt_yaw", 100, tgtYawCallback);

	gz::transport::Node gz_node;
	RCLCPP_INFO(node->get_logger(), "Broadcasting F450 + aruco_Target via %s", service.c_str());

	tf2::Quaternion q_rot;
	q_rot.setRPY(3.141592, 0, 0);

	while (rclcpp::ok())
	{
		gz::msgs::Pose_V msg;

		// --- F450: pi roll pre-rotation, camera ends up looking down -------------
		tf2::Quaternion q_att;
		q_att.setRPY(roll, pitch, yaw);
		tf2::Quaternion q_quad = q_rot * q_att;
		q_quad.normalize();
		fill(msg.add_pose(), "F450", quad_x, -quad_y, -quad_z, q_quad);

		// --- aruco_Target: yaw only, no pre-rotation -----------------------------
		tf2::Quaternion q_tgt;
		q_tgt.setRPY(0, 0, -tgt_yaw);
		q_tgt.normalize();
		fill(msg.add_pose(), "aruco_Target", tgt_x, -tgt_y, -tgt_z, q_tgt);

		gz_node.Request(service, msg, &onSetPose);

		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}

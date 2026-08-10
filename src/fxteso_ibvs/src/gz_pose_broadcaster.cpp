#include <rclcpp/rclcpp.hpp>
#include "fxteso_ibvs/sim_rate.hpp"
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

float quad_x = 0, quad_y = 0, quad_z = 0;
float roll = 0, pitch = 0, yaw = 0;

float tgt_x = 0, tgt_y = 0, tgt_z = 0;
float tgt_yaw = 0;


bool have_quad_pos = false, have_quad_att = false;
bool have_tgt_pos = false, have_tgt_yaw = false;


float quad_thrust = 2.0f * 9.81f;

void quadPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr p)
{
	quad_x = p->x;
	quad_y = p->y;
	quad_z = p->z;
	have_quad_pos = true;
}

void quadAttCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr a)
{
	roll = a->x;
	pitch = a->y;
	yaw = a->z;
	have_quad_att = true;
}

void tgtPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr p)
{
	tgt_x = p->x;
	tgt_y = p->y;
	tgt_z = p->z;
	have_tgt_pos = true;
}

void tgtYawCallback(const std_msgs::msg::Float64::ConstSharedPtr y)
{
	tgt_yaw = y->data;
	have_tgt_yaw = true;
}

void quadThrustCallback(const std_msgs::msg::Float64::ConstSharedPtr t)
{
	quad_thrust = t->data;
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

	// Rotor speed is cosmetic, and chosen to look right rather than be right:
	// a real ~500 rad/s just aliases at 50-60 fps.
	const double hover_rate = node->declare_parameter<double>("rotor_hover_rate", 60.0);
	const double idle_rate = node->declare_parameter<double>("rotor_idle_rate", 12.0);
	const double hover_thrust = node->declare_parameter<double>("rotor_hover_thrust", 19.62);

	// Under plant:=gazebo physics owns the quad's pose; the target is always broadcast.
	const bool teleport_quad = node->declare_parameter<bool>("teleport_quad", true);

	fxteso::SimRate loop_rate(node, 100);

	auto quad_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_position", 100, quadPosCallback);
	auto quad_att_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"quad_attitude", 100, quadAttCallback);
	auto tgt_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>(
		"tgt_position", 100, tgtPosCallback);
	auto tgt_yaw_sub = node->create_subscription<std_msgs::msg::Float64>(
		"tgt_yaw", 100, tgtYawCallback);
	auto thrust_sub = node->create_subscription<std_msgs::msg::Float64>(
		"quad_thrust", 1, quadThrustCallback);

	gz::transport::Node gz_node;
	RCLCPP_INFO(node->get_logger(), "Broadcasting %s via %s",
		teleport_quad ? "F450 + aruco_Target" : "aruco_Target only (F450 pose owned by physics)",
		service.c_str());

	// Diagonal pairs turn together, adjacent rotors oppose. The sign is the direction the
	// JointController plugins in models/F450/model.sdf declare as their initial velocity;
	// keep the two in agreement or the rotors reverse when the controllers come up.
	const std::array<std::pair<const char *, double>, 4> rotors = {{
		{"rotor_front_left_joint", -1.0},
		{"rotor_rear_right_joint", -1.0},
		{"rotor_front_right_joint", 1.0},
		{"rotor_rear_left_joint", 1.0},
	}};

	std::array<gz::transport::Node::Publisher, 4> rotor_pubs;
	for (size_t i = 0; i < rotors.size(); ++i)
	{
		rotor_pubs[i] = gz_node.Advertise<gz::msgs::Double>(
			std::string("/model/F450/joint/") + rotors[i].first + "/cmd_vel");
	}

	unsigned tick = 0;

	while (rclcpp::ok())
	{
		gz::msgs::Pose_V msg;

		// ROS NED -> gz is a conjugation by Rx(pi): (x,-y,-z) and RPY(r,-p,-y).
		if (teleport_quad && have_quad_pos && have_quad_att)
		{
			tf2::Quaternion q_quad;
			q_quad.setRPY(roll, -pitch, -yaw);
			q_quad.normalize();
			fill(msg.add_pose(), "F450", quad_x, -quad_y, -quad_z, q_quad);
		}

		// aruco_Target: yaw only, no pre-rotation
		if (have_tgt_pos && have_tgt_yaw)
		{
			tf2::Quaternion q_tgt;
			q_tgt.setRPY(0, 0, -tgt_yaw);
			q_tgt.normalize();
			fill(msg.add_pose(), "aruco_Target", tgt_x, -tgt_y, -tgt_z, q_tgt);
		}

		if (msg.pose_size() > 0)
			gz_node.Request(service, msg, &onSetPose);

		// Rotor speeds: thrust ~ omega^2. 20 Hz is plenty for a cosmetic signal.
		if (tick++ % 5 == 0)
		{
			const double ratio = std::max(0.0, static_cast<double>(quad_thrust)) / hover_thrust;
			const double omega = std::clamp(hover_rate * std::sqrt(ratio),
			                                idle_rate, 2.0 * hover_rate);

			for (size_t i = 0; i < rotors.size(); ++i)
			{
				gz::msgs::Double cmd;
				cmd.set_data(rotors[i].second * omega);
				rotor_pubs[i].Publish(cmd);
			}
		}

		loop_rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}

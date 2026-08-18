#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include <eigen3/Eigen/Dense>


bool control_started = false;

float pos_x;
float pos_y;
float xp;
float yp;
float zp;
float xpp;
float ypp;
float pos_z;
float yaw;
float yawRate;
float yawAccel;
float t;
float step = 0.01;
float arg;
float x_traj;
float y_traj;

Eigen::Vector3f quad_pos;
Eigen::Vector3f error;


void quadPosCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr quadPos)
{
	quad_pos(0) = quadPos->x;
    quad_pos(1) = quadPos->y;
    quad_pos(2) = quadPos->z;
}


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("tgt_pos");

	fxteso::SimRate loop_rate(node, 100);
	
	auto tgt_pos_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_position",100);
	auto tgt_yaw_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw",100);
	auto tgt_vel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_velocity",100);
	auto tgt_yaw_rate_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw_rate",100);
	auto tgt_accel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("tgt_acceleration",100);
	auto tgt_yaw_acceleration_pub = node->create_publisher<std_msgs::msg::Float64>("tgt_yaw_acceleration",100);

	auto error_lin_pub = node->create_publisher<geometry_msgs::msg::Vector3>("position_error",100);
	

	auto quad_pos_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_position", 1, quadPosCallback);

	// The trajectory starts 4.35 s in, which assumes the aircraft is already over the marker.
	// plant:=px4 spends ~25 s on EKF2, arming and takeoff, by which time the target has driven
	// out of the camera footprint. This holds it on its start pose until pos_ctrl publishes.
	// Default false, so analytic and gazebo keep their recorded timing.
	const bool hold_until_control =
		node->declare_parameter<bool>("hold_until_control", false);

	auto ctrl_sub = node->create_subscription<geometry_msgs::msg::Quaternion>(
		"desired_attitude", 1,
		[](const geometry_msgs::msg::Quaternion::ConstSharedPtr) { control_started = true; });



	geometry_msgs::msg::Vector3 tgt_position;
	geometry_msgs::msg::Vector3 positionError;
	std_msgs::msg::Float64 tgt_yaw;
	
	geometry_msgs::msg::Vector3 tgt_vel;
	std_msgs::msg::Float64 tgt_yaw_vel;
	std_msgs::msg::Float64 tgt_psi_rate;

	geometry_msgs::msg::Vector3 tgt_accel;
	std_msgs::msg::Float64 tgt_yaw_accel;	
	
	
	int i = 0;
	int sim_time = 100/step; //Seconds / step

	//Tgt_pert
	pos_x = -10;
	pos_y = -10;
	//pos_x = 0;
	//pos_y = 0;
	pos_z = 0;
	yaw = 0;

	xp = 0;
	yp = 0;
	zp = 0;
	yawRate = 0;

	tgt_position.x = pos_x;
	tgt_position.y = pos_y;
	tgt_position.z = pos_z;
	
	tgt_yaw.data = yaw;
	
	tgt_vel.x = xp;
	tgt_vel.y = yp;
	tgt_vel.z = 0;

	tgt_psi_rate.data = yawRate;
	
	loop_rate.sleepFor(0.05);
	tgt_pos_pub->publish(tgt_position);
	tgt_yaw_pub->publish(tgt_yaw);
	tgt_vel_pub->publish(tgt_vel);
	tgt_yaw_rate_pub->publish(tgt_psi_rate);
	loop_rate.sleepFor(2.3);
	tgt_pos_pub->publish(tgt_position);
	tgt_yaw_pub->publish(tgt_yaw);
	tgt_vel_pub->publish(tgt_vel);
	tgt_yaw_rate_pub->publish(tgt_psi_rate);

	loop_rate.sleepFor(2.0);

	if (hold_until_control)
		RCLCPP_INFO(node->get_logger(),
		            "Holding the target on its start pose until desired_attitude appears.");

	while(rclcpp::ok())
	{
		t = i*step;
		/*
		pos_x = 0;
		pos_y = 0;
		pos_z = 0;
		yaw = 0;

		xp = 0;
		yp = 0;
		zp = 0;
		yawRate = 0;
		*/		
		/*
		xp = 0;
		yp = 0;
		yawRate = 0;
		*/
			
		if(t>=0 && t<2)
		{
			xp = 0;
			xp = 0;
			yawRate = 0;
		}

		else if (t >= 2 && t < 5)
		{
			xp = 0.1;
			yp = 0;
			yawRate = 0;
		}	
		else if (t >= 5 && t < 8)
		{
			xp = 0.2;
			yp = 0;
			yawRate = 0;
		}
		else if (t >= 8 && t < 11)
		{
			xp = 0.3;
			yp = 0;
			yawRate = 0;
		}
		else if (t >= 11 && t < 14)
		{
			xp = 0.4;
			yp = 0;
			yawRate = 0;
		}	
		else if (t >= 14 && t < 17)
		{
			xp = 0.5;
			yp = 0;
			yawRate = 0;
		}
		else if (t >= 17 && t < 20)
		{
			xp = 0.6;
			yp = 0;
			yawRate = 0;
		}		
		else if (t >= 20 && t < 23)
		{
			xp = 0.7;
			yp = 0;
			yawRate = 0;
		}
		else if (t >= 23 && t < 26)
		{
			xp = 0.8;
			yp = 0;
			yawRate = 0;
		}
		else if (t >= 26 && t < 30)
		{
			xp = 0.9;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=30 && t<124.2478)
		{	
			xp = 1 * cos(0.1*(t-30));
			yp = 1 * sin(0.1*(t-30));	
			yawRate = 0.1;

			if (t>=70 && t<100)
			{
				pos_z = sin(0.05*3.141592*(t-30));
			}
			else
			{
				pos_z = 0;
			}

			if (pos_z < 0)
			{
				pos_z = 0;
			}

			if (pos_z>0.5)
			{
				pos_z = 0.5;
			}

		}
		else if (t>=124.2478 && t<140)
		{
			xp = -1;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=140 && t<143)
		{
			xp = -0.8;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=143 && t<146)
		{
			xp = -0.7;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=146 && t<149)
		{
			xp = -0.6;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=149 && t<152)
		{
			xp = -0.5;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=152 && t<155)
		{
			xp = -0.4;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=155 && t<158)
		{
			xp = -0.3;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=158 && t<162)
		{
			xp = -0.2;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=158 && t<165)
		{
			xp = -0.1;
			yp = 0;
			yawRate = 0;
		}
		else if (t>=165)
		{
			xp = 0;
			yp = 0;
			yawRate = 0;
		}
		
		pos_x = pos_x + xp * step;
		pos_y = pos_y + yp * step;
		yaw = yaw + yawRate * step;	
		

		error(0) = quad_pos(0) - pos_x;
		error(1) = quad_pos(1) - pos_y;
		error(2) = quad_pos(2) + 2.5 + pos_z;

		tgt_position.x = pos_x;
		tgt_position.y = pos_y;
		tgt_position.z = -pos_z;
	
		tgt_yaw.data = yaw;

		tgt_vel.x = xp;
		tgt_vel.y = yp;
		tgt_vel.z = 0;
		tgt_psi_rate.data = yawRate;

		tgt_accel.x = 0;
		tgt_accel.y = 0;
		tgt_accel.z = 0;
		tgt_yaw_accel.data = 0;


		positionError.x = error(0);
		positionError.y = error(1);
		positionError.z = error(2);

		tgt_pos_pub->publish(tgt_position);
		tgt_yaw_pub->publish(tgt_yaw);
		
		tgt_vel_pub->publish(tgt_vel);
		tgt_yaw_rate_pub->publish(tgt_psi_rate);

		tgt_accel_pub->publish(tgt_accel);

		error_lin_pub->publish(positionError);
		

		// Freezing the clock rather than the outputs keeps the trajectory itself untouched:
		// it simply starts later, from the same t=0 it always did.
		if (!hold_until_control || control_started)
			i = i+1;

		std::cout << yawRate << std::endl;

		loop_rate.sleep();
		
		
	}
	rclcpp::shutdown();
	return 0;	
}

//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

float step = 0.01;
float gravity = 9.81;
float quad_mass = 2;
float thrust = quad_mass*gravity;

Eigen::Vector3f linear_position;
Eigen::Vector3f attitude_position;

Eigen::Vector3f linear_velocity_BF;
Eigen::Vector3f linear_velocity_IF;
Eigen::Vector3f attitude_velocity;

Eigen::Vector3f linear_acceleration_BF;
Eigen::Vector3f attitude_acceleration;

Eigen::Vector3f tau;
Eigen::Vector3f dist;

Eigen::Vector3f force(0,0,0);

Eigen::Vector3f e3(0,0,1);

Eigen::Matrix3f J;

Eigen::Matrix3f RotationMatrix(Eigen::Vector3f attitude)
{
	float cos_phi = cos(attitude(0));
	float sin_phi = sin(attitude(0));
	
	float cos_theta = cos(attitude(1));
	float sin_theta = sin(attitude(1));
	
	
	float cos_psi = cos(attitude(2));
	float sin_psi = sin(attitude(2));
	
	
	Eigen::Matrix3f R;
	
	R << cos_psi * cos_theta, cos_psi * sin_phi * sin_theta - cos_phi * sin_psi, sin_psi * sin_phi + cos_psi * cos_phi * sin_theta,
			 cos_theta * sin_psi, cos_psi * cos_phi + sin_psi * sin_phi * sin_theta, cos_phi * sin_psi * sin_theta - cos_psi * sin_phi,
			 -sin_theta, cos_theta * sin_phi, cos_phi * cos_theta;
			 
	return R;
	
}

Eigen::Matrix3f skewOmega(Eigen::Vector3f attitude_velocity)
{
	Eigen::Matrix3f Skew;
	
	Skew << 0, -attitude_velocity(2), attitude_velocity(1),
					attitude_velocity(2), 0, -attitude_velocity(0),
					-attitude_velocity(1), attitude_velocity(0), 0;
					
	return Skew;

}

void ThrustInputCallback(const std_msgs::msg::Float64::ConstSharedPtr thr)
{
	thrust = thr->data;
}

void TorqueInputsCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr tor)
{
	tau(0) = tor->x;
	tau(1) = tor->y;
	tau(2) = tor->z;
}

void DisturbancesCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr d)
{
	dist(0) = d->x;
	dist(1) = d->y;
	dist(2) = d->z;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("uav_dynamics");

	fxteso::SimRate loop_rate(node, 100);
	
	auto thrust_sub = node->create_subscription<std_msgs::msg::Float64>("quad_thrust", 1, ThrustInputCallback);
	auto quad_torques_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_torques", 1, TorqueInputsCallback);
	auto disturbances_sub = node->create_subscription<geometry_msgs::msg::Vector3>("disturbances", 1, DisturbancesCallback);
	
	geometry_msgs::msg::Vector3 positionVector;
	geometry_msgs::msg::Vector3 attitudeVector;
	geometry_msgs::msg::Vector3 velocityVector;
	geometry_msgs::msg::Vector3 att_velVector;
	geometry_msgs::msg::Vector3 BFvelocityVector;
	
	auto positionPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_position",100);
	auto quad_attitude_pub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude",100);
	auto velocityPub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity",100);
	auto quad_attitude_velocity_pub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_attitude_velocity",100);
	auto quad_vel_BF_pub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_velocity_BF",100);
	
	
	J << 0.0411, 0, 0,
		 0, 0.0478, 0,
		 0, 0, 0.0599;
	
	// x,y start 0.14 m from the target, which target_position.cpp initialises at (-10,-10).
	// z = -4 is the paper's initial condition.
	const double start_alt = node->declare_parameter<double>("start_altitude", 4.0);
	linear_position << -9.9, -10.1, -start_alt;
	
	attitude_position << 0,0,0;

	linear_velocity_BF << 0,0,0;
	linear_velocity_IF << 0,0,0;
	attitude_velocity << 0,0,0;

	linear_acceleration_BF << 0,0,0;
	attitude_acceleration << 0,0,0;

		 
	positionVector.x = linear_position(0);
	positionVector.y = linear_position(1);
	positionVector.z = linear_position(2);
	
	attitudeVector.x = attitude_position(0);
	attitudeVector.y = attitude_position(1);
	attitudeVector.z = attitude_position(2);

	BFvelocityVector.x = linear_velocity_BF(0);
	BFvelocityVector.y = linear_velocity_BF(1);
	BFvelocityVector.z = linear_velocity_BF(2);
	
	positionPub->publish(positionVector);
	quad_attitude_pub->publish(attitudeVector);
	quad_vel_BF_pub->publish(BFvelocityVector);
	//rclcpp::sleep_for(std::chrono::milliseconds(50));
	positionPub->publish(positionVector);
	quad_attitude_pub->publish(attitudeVector);
	quad_vel_BF_pub->publish(BFvelocityVector);
	loop_rate.sleepFor(5.0);
	
	// Explicit forward Euler at 100 Hz: it diverges to 1e10 m within seconds of losing
	// closed-loop visual feedback, so this plant is only valid while lock holds.
	while(rclcpp::ok())
	{
		//Angular dynamics
		attitude_acceleration = J.inverse() * (tau - (skewOmega(attitude_velocity) * J * attitude_velocity));
	
		for(int i = 0; i <= 2; i++)
		{
			attitude_velocity(i) = attitude_velocity(i) + step * attitude_acceleration(i);
			attitude_position(i) = attitude_position(i) + step * attitude_velocity(i);
		}
		
		//Linear dynamics
		force = (RotationMatrix(attitude_position).inverse() * (quad_mass * gravity * e3)) - (thrust * e3) - RotationMatrix(attitude_position).inverse() * dist; 
		
		linear_acceleration_BF = force/quad_mass - skewOmega(attitude_velocity) * linear_velocity_BF; 
		 
		for(int i = 0; i <= 2; i++)
		{
			linear_velocity_BF(i) = linear_velocity_BF(i) + step * linear_acceleration_BF(i);
		}
		
		linear_velocity_IF = RotationMatrix(attitude_position) * linear_velocity_BF;
		
		for(int i = 0; i <= 2; i++)
		{
			linear_position(i) = linear_position(i) + step * linear_velocity_IF(i);
		}

		if(linear_position(2)>0)
		{
			linear_position(2) = 0;
		}
		
		positionVector.x = linear_position(0);
		positionVector.y = linear_position(1);
		positionVector.z = linear_position(2);
		
		attitudeVector.x = attitude_position(0);
		attitudeVector.y = attitude_position(1);
		attitudeVector.z = attitude_position(2);
		
		velocityVector.x = linear_velocity_IF(0);
		velocityVector.y = linear_velocity_IF(1);
		velocityVector.z = linear_velocity_IF(2);
		
		att_velVector.x = attitude_velocity(0);
		att_velVector.y = attitude_velocity(1);
		att_velVector.z = attitude_velocity(2);
		
		BFvelocityVector.x = linear_velocity_BF(0);
		BFvelocityVector.y = linear_velocity_BF(1);
		BFvelocityVector.z = linear_velocity_BF(2);
		
		positionPub->publish(positionVector);
		quad_attitude_pub->publish(attitudeVector);
		velocityPub->publish(velocityVector);
		quad_attitude_velocity_pub->publish(att_velVector);
		quad_vel_BF_pub->publish(BFvelocityVector);
		
		//std::cout << "U1 : " << U1 << std::endl;
		std::cout << "force: " << force(0) << std::endl; 
		
		loop_rate.sleep();
				
	}
	rclcpp::shutdown();
	return 0;	
}

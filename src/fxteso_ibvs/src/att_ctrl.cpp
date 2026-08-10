//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "fxteso_ibvs/sim_rate.hpp"
#include <chrono>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>

Eigen::Vector3f attVelRef(0,0,0);
Eigen::Vector3f attRef(0,0,0);
Eigen::Vector3f error_att(0,0,0);
Eigen::Vector3f error_attVel(0,0,0);

Eigen::Vector3f attitude(0,0,0);
Eigen::Vector3f attitudeEstimates(0,0,0);

Eigen::Vector3f attitude_vel(0,0,0);
Eigen::Vector3f attitudeVelEstimates(0,0,0);

Eigen::Vector3f angularDisturbanceEstimates(0,0,0);

Eigen::Vector3f sigma(0,0,0);
Eigen::Vector3f xi_1(0,0,0);
Eigen::Vector3f xi_2(0,0,0);
Eigen::Vector3f gam(0,0,0);
Eigen::Vector3f lambda(0,0,0);

Eigen::Vector3f asmc(0,0,0);
Eigen::Vector3f k(0,0,0);
Eigen::Vector3f k_dot(0,0,0);
Eigen::Vector3f alpha(0,0,0);
Eigen::Vector3f beta(0,0,0);

Eigen::Vector3f tau;

float step_size = 0.01;

float Jxx = 0.0411;
float Jyy = 0.0478;
float Jzz = 0.0599;


float sign(float var)
{   
    float x;
    if (var > 0)
    {
        x = 1;
    }
    else if (var<0)
    {
        x = -1;
    }
    else if (var == 0)
    {
        x = 0;
    }
    return x;
}

/*void attDesCallback(const geometry_msgs::msg::Quaternion::ConstSharedPtr aD)
{
    attRef(0) = aD->x;
    attRef(1) = aD->y;
    attRef(2) = aD->z;
    attVelRef(2) = aD->w;
}*/


void attCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr a)
{
    attitude(0) = a->x;
    attitude(1) = a->y;
    attitude(2) = a->z;
}

void attVelCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr av)
{
    attitude_vel(0) = av->x;
    attitude_vel(1) = av->y;
    attitude_vel(2) = av->z;
}

void attEstCallback(const geometry_msgs::msg::Twist::ConstSharedPtr aE)
{
    attitudeEstimates(0) = aE->linear.x;
    attitudeEstimates(1) = aE->linear.y;
    attitudeEstimates(2) = aE->linear.z;
    
    attitudeVelEstimates(0) = aE->angular.x;
    attitudeVelEstimates(1) = aE->angular.y;
    attitudeVelEstimates(2) = aE->angular.z;
}

void attDesEstCallback(const geometry_msgs::msg::Twist::ConstSharedPtr adE)
{
    attRef(0) = adE->linear.x;
    attRef(1) = adE->linear.y;
    attRef(2) = adE->linear.z;
    
    attVelRef(0) = adE->angular.x;
    attVelRef(1) = adE->angular.y;
    attVelRef(2) = adE->angular.z;
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("att_ctrl");

	fxteso::SimRate loop_rate(node, 100);	

    //auto att_des_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("desired_attitude", 1, attDesCallback);
  	auto att_des_est_sub = node->create_subscription<geometry_msgs::msg::Twist>("attitude_desired_estimates", 1, attDesEstCallback);
    
    auto att_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_attitude", 1, attCallback);
    auto att_est_sub = node->create_subscription<geometry_msgs::msg::Twist>("attitude_estimates", 1, attEstCallback);
    auto attVel_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_attitude_velocity", 1, attVelCallback);

    auto torque_pub = node->create_publisher<geometry_msgs::msg::Vector3>("quad_torques",100);
    auto sigma_att_pub = node->create_publisher<geometry_msgs::msg::Vector3>("sigma_att",100);
    auto error_att_pub = node->create_publisher<geometry_msgs::msg::Vector3>("error_att",100);
    auto error_attVel_pub = node->create_publisher<geometry_msgs::msg::Vector3>("error_attVel",100);
    auto k_att_pub = node->create_publisher<geometry_msgs::msg::Vector3>("k_att",100);

    geometry_msgs::msg::Vector3 torque_var;
    geometry_msgs::msg::Vector3 sigma_att_var;
    geometry_msgs::msg::Vector3 error_att_var;
    geometry_msgs::msg::Vector3 error_attVel_var;
    geometry_msgs::msg::Vector3 k_att_var;

    xi_1 << 3, 3, 3;
    xi_2 << 0.3, 0.3, 0.5;
    lambda << 1.5, 1.5, 1.5;
    gam << 1.3, 1.3, 1.3;
    k << 0,0,0;
    k_dot << 0,0,0;
    alpha << 8, 8, 5;
    beta << 2, 2, 3;

    //attVelRef(0) = 0;
    //attVelRef(1) = 0;
    
    torque_var.x = 0;
    torque_var.y = 0;
    torque_var.z = 0;

    sigma_att_var.x = 0;
    sigma_att_var.y = 0;
    sigma_att_var.z = 0;

    error_att_var.x = 0;
    error_att_var.y = 0;
    error_att_var.z = 0;

    error_attVel_var.x = 0;
    error_attVel_var.y = 0;
    error_attVel_var.z = 0;
        
    k_att_var.x = k(0);
    k_att_var.y = k(1);
    k_att_var.z = k(2);
        
    k_att_pub->publish(k_att_var);
    torque_pub->publish(torque_var);
    sigma_att_pub->publish(sigma_att_var);
    error_att_pub->publish(error_att_var); 
    error_attVel_pub->publish(error_attVel_var);   

    loop_rate.sleepFor(2.0);

    while(rclcpp::ok())
    { 
        //Error
        error_att = attitudeEstimates - attRef;
        error_attVel= attitudeVelEstimates - attVelRef;

        // Shortest angular route on yaw. Identity while |error| < pi, so the control law is
        // unchanged in normal flight. See docs/angle-wrapping.md.
        while (error_att(2) > M_PI)  error_att(2) -= 2.0f * M_PI;
        while (error_att(2) <= -M_PI) error_att(2) += 2.0f * M_PI;

        //error_att = attitude - attRef;
        //error_attVel= attitude_vel - attVelRef;

        //Nonsingular terminal sliding surface
        for (int i = 0; i <= 2; i++)
        {
            sigma(i) = error_att(i) + xi_1(i) * powf(std::abs(error_att(i)),lambda(i)) * sign(error_att(i)) + xi_2(i) * powf(std::abs(error_attVel(i)),(gam(i))) * sign(error_attVel(i));

            k_dot(i) = powf(alpha(i),0.5) * powf(std::abs(sigma(i)),0.5) - powf(beta(i),0.5) * powf(k(i),2);

            k(i) = k(i) + k_dot(i) * step_size;

            asmc(i) = -2*k(i) * powf(std::abs(sigma(i)),0.5) * sign(sigma(i)) - (powf(k(i),2)/2) * sigma(i);
        }

        


/*
        tau(0) = Jxx * ( - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0))*xi_1(0)*lambda(0)*powf(std::abs(error_att(0)), lambda(0)-1) + asmc(0) );
        tau(1) = Jyy * ( - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1))*xi_1(1)*lambda(1)*powf(std::abs(error_att(1)), lambda(1)-1) + asmc(1) );
        tau(2) = Jzz * ( - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2))*xi_1(2)*lambda(2)*powf(std::abs(error_att(2)), lambda(2)-1) + asmc(2) );
*/

        tau(0) = Jxx * ( -(((Jyy-Jzz)/Jxx)*attitudeVelEstimates(1)*attitudeVelEstimates(2)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0)) - (1/(xi_2(0)*gam(0)))*powf(std::abs(error_attVel(0)), 2-gam(0))*sign(error_attVel(0))*xi_1(0)*lambda(0)*powf(std::abs(error_att(0)), lambda(0)-1) + asmc(0) );
        tau(1) = Jyy * ( -(((Jzz-Jxx)/Jyy)*attitudeVelEstimates(0)*attitudeVelEstimates(2)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1)) - (1/(xi_2(1)*gam(1)))*powf(std::abs(error_attVel(1)), 2-gam(1))*sign(error_attVel(1))*xi_1(1)*lambda(1)*powf(std::abs(error_att(1)), lambda(1)-1) + asmc(1) );
        tau(2) = Jzz * ( -(((Jxx-Jyy)/Jzz)*attitudeVelEstimates(0)*attitudeVelEstimates(1)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2)) - (1/(xi_2(2)*gam(2)))*powf(std::abs(error_attVel(2)), 2-gam(2))*sign(error_attVel(2))*xi_1(0)*lambda(2)*powf(std::abs(error_att(2)), lambda(2)-1) + asmc(2) );

        torque_var.x = tau(0);
        torque_var.y = tau(1);
        torque_var.z = tau(2);

        sigma_att_var.x = sigma(0);
        sigma_att_var.y = sigma(1);
        sigma_att_var.z = sigma(2);

        error_att_var.x = error_att(0);
        error_att_var.y = error_att(1);
        error_att_var.z = error_att(2);

        error_attVel_var.x = error_attVel(0);
        error_attVel_var.y = error_attVel(1);
        error_attVel_var.z = error_attVel(2);
        
        k_att_var.x = k(0);
        k_att_var.y = k(1);
        k_att_var.z = k(2);
        
        k_att_pub->publish(k_att_var);
        torque_pub->publish(torque_var);
        sigma_att_pub->publish(sigma_att_var);
        error_att_pub->publish(error_att_var);
        error_attVel_pub->publish(error_attVel_var);
       
	loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

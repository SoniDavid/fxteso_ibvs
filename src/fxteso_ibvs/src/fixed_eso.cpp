//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "fxteso_ibvs/sim_rate.hpp"
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

float step = 0.02;

Eigen::Vector4f imFeat; //qx, qy, qz, qpsi
Eigen::Vector4f ctrl_input; //vDot_x, vDot_y, vDot_z, vDot_yaw
Eigen::Vector4f imFeat_estimate(0,0,1,0);
Eigen::Vector4f imFeat_estimate_dot(0,0,0,0);
Eigen::Vector4f ibvs_dist(0,0,0,0);
Eigen::Vector4f estimation_error(0,0,0,0);
Eigen::Vector3f scaled_dist(0,0,0);

Eigen::Vector4f x1_hat(0,0,0,0);
Eigen::Vector4f x2_hat(0,0,0,0);
Eigen::Vector4f x3_hat(0,0,0,0);

Eigen::Vector4f x1_hat_dot(0,0,0,0);
Eigen::Vector4f x2_hat_dot(0,0,0,0);
Eigen::Vector4f x3_hat_dot(0,0,0,0);

Eigen::Vector4f gamma1;
Eigen::Vector4f gamma2;
Eigen::Vector4f gamma3;
Eigen::Vector4f gamma4;

Eigen::Vector4f mu1;
Eigen::Vector4f mu2;
Eigen::Vector4f mu3;

float alpha = 0;
float alpha2 = 0;
float alpha3 = 0;

float beta = 0;
float beta2 = 0;
float beta3 = 0;

float fx = 0;
float gx_u = 0;
float quad_mass = 2;
float z_des = 2.5;

float yawRate_desired = 0;

Eigen::Vector3f quad_att(0,0,0);
Eigen::Vector3f dist_linear(0,0,0);

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
}

float sign(float var)
{
    float result;
    if(var>0)
    {
        result = 1;
    }
    else if(var<0)
    {
        result = -1;
    }
    else if (var == 0)
    {
        result = 0;
    }
    return result;
}

float sig(float a, float b) //sig = |a|^b * sign(a)
{
    float s;
    s = powf(std::abs(a), b) * sign(a);
    return s;
}

void quadAttCallback(const geometry_msgs::msg::Vector3::ConstSharedPtr quadAtt)
{
	quad_att(0) = quadAtt->x;
    quad_att(1) = quadAtt->y;
    quad_att(2) = quadAtt->z;
}

void imFeatCallback(const geometry_msgs::msg::Quaternion::ConstSharedPtr img_features)
{
	imFeat(0) = img_features->x;
	imFeat(1) = img_features->y;
	imFeat(2) = img_features->z;
    imFeat(3) = img_features->w;
}

void ibvsCtrlCallback(const geometry_msgs::msg::Quaternion::ConstSharedPtr ctrl)
{
	ctrl_input(0) = ctrl->x;
	ctrl_input(1) = ctrl->y;
	ctrl_input(2) = ctrl->z;
    ctrl_input(3) = ctrl->w;
}

void yawRateCallback(const geometry_msgs::msg::Quaternion::ConstSharedPtr yr)
{
	yawRate_desired = yr->w;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("fixed_eso");

	fxteso::SimRate loop_rate(node, 50);	

    //ROS publishers and subscribers
    auto im_feat_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("ImFeat_vector", 1, imFeatCallback);  
    auto ctrl_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("ibvs_control_input", 1, ibvsCtrlCallback);  
    auto yawRate_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("desired_attitude", 1, yawRateCallback);
    auto quad_att_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_attitude", 1, quadAttCallback);

    auto im_feat_est_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("ImFeat_estimates_fxt",100);
    auto im_feat_est_err_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("estimation_error_fxt",100);
    auto im_feat_est_dot_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("ImFeat_dot_estimates_fxt",100);
    auto dist_est_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("ibvs_dist",100);
    auto scaled_dist_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("scaled_ibvs_dist",100);

    geometry_msgs::msg::Quaternion im_feat_est_var;
    geometry_msgs::msg::Quaternion im_feat_est_dot_var;
    geometry_msgs::msg::Quaternion im_feat_est_err_var;
    geometry_msgs::msg::Quaternion ibvs_dist_var;
    geometry_msgs::msg::Quaternion scaled_ibvs_dist_var;

    imFeat_estimate << imFeat(0), imFeat(1), 1.7, 0;
    imFeat_estimate_dot << 0,0,0,0;
    ibvs_dist << 0, 0, 0, 0;

    estimation_error << imFeat(0), imFeat(1), 0, 0;
    x1_hat_dot << 0, 0, 0, 0;
    x2_hat_dot << 0, 0, 0, 0;
    x3_hat_dot << 0, 0, 0, 0;

    // //GAINS WITH MODEL UNCERTAINTIES
   gamma1 << 18, 18, 16, 5;
   gamma2 << 10, 10, 14, 16;
   gamma3 << 7, 7, 21, 7;
   gamma4 << 0.001, 0.001, 0.001, 0.001;
  
     /*
     
    gamma1 << 16, 16, 16, 12;
   gamma2 << 14, 14, 14, 14;
   gamma3 << 6, 6, 21, 3;
   gamma4 << 0.0001, 0.0001, 0.001, 0.001;
*/


    
    mu1 = gamma1;
    mu2 = gamma2;
    mu3 = gamma3;
    

    im_feat_est_var.x = imFeat_estimate(0);
    im_feat_est_var.y = imFeat_estimate(1);
    im_feat_est_var.z = imFeat_estimate(2);
    im_feat_est_var.w = imFeat_estimate(3);

    im_feat_est_dot_var.x = imFeat_estimate_dot(0);
    im_feat_est_dot_var.y = imFeat_estimate_dot(1);
    im_feat_est_dot_var.z = imFeat_estimate_dot(2);
    im_feat_est_dot_var.w = imFeat_estimate_dot(3);

    im_feat_est_err_var.x = estimation_error(0);
    im_feat_est_err_var.y = estimation_error(1);
    im_feat_est_err_var.z = estimation_error(2);
    im_feat_est_err_var.w = estimation_error(3);

    ibvs_dist_var.x = 0;
    ibvs_dist_var.y = 0;
    ibvs_dist_var.z = 0;
    ibvs_dist_var.w = 0;

    scaled_ibvs_dist_var.x = 0;
    scaled_ibvs_dist_var.y = 0;
    scaled_ibvs_dist_var.z = 0;
    scaled_ibvs_dist_var.w = 0;

    im_feat_est_pub->publish(im_feat_est_var);
    im_feat_est_dot_pub->publish(im_feat_est_dot_var);
    im_feat_est_err_pub->publish(im_feat_est_err_var);
    dist_est_pub->publish(ibvs_dist_var);
    scaled_dist_pub->publish(scaled_ibvs_dist_var);

    loop_rate.sleepFor(4.7);

    while(rclcpp::ok())
    {        
        for(int i = 0; i<=3; i++)
        {
            estimation_error(i) = imFeat(i) - imFeat_estimate(i);

            //Defining f(x) and g(x)u
            if(i == 0)
            {   
                alpha = 0.75;
                beta = 1.2;

                alpha2 = (alpha + 1)/2;
                beta2 = (beta + 1)/2;
    
                alpha3 = (alpha + 2)/3;
                beta3 = (beta + 2)/3;

                fx =0; //imFeat_estimate_dot(1) * yawRate_desired + ctrl_input(3) * imFeat_estimate(1);
                gx_u = -(1/z_des)*ctrl_input(0);
            }
            else if (i == 1)
            {
                alpha = 0.75;
                beta = 1.2;

                alpha2 = (alpha + 1)/2;
                beta2 = (beta + 1)/2;
    
                alpha3 = (alpha + 2)/3;
                beta3 = (beta + 2)/3;

                fx = 0;//-imFeat_estimate_dot(0) * yawRate_desired - ctrl_input(3) * imFeat_estimate(0);
                gx_u = -(1/z_des)*ctrl_input(1);
            }
            else if (i == 2)
            {
                alpha = 0.75;
                beta = 1.2;

                alpha2 = (alpha + 1)/2;
                beta2 = (beta + 1)/2;
    
                alpha3 = (alpha + 2)/3;
                beta3 = (beta + 2)/3;
                fx = 0;
                gx_u = -(1/z_des)*ctrl_input(2);
            }
            else if (i == 3)
            {
                alpha = 0.75;
                beta = 1.2;

                alpha2 = (alpha + 1)/2;
                beta2 = (beta + 1)/2;
    
                alpha3 = (alpha + 2)/3;
                beta3 = (beta + 2)/3;
                fx = 0;
                gx_u = ctrl_input(3);
            }

            x3_hat_dot(i) = gamma3(i) * sig(estimation_error(i),alpha) + mu3(i) * sig(estimation_error(i),beta) + gamma4(i) * sign(estimation_error(i));
            ibvs_dist(i) = ibvs_dist(i) + x3_hat_dot(i) * step;

            x2_hat_dot(i) = ibvs_dist(i) + fx + gx_u + gamma2(i)*sig(estimation_error(i), alpha2) + mu2(i)*sig(estimation_error(i), beta2);
            imFeat_estimate_dot(i) = imFeat_estimate_dot(i) + x2_hat_dot(i)*step;

            x1_hat_dot(i) = imFeat_estimate_dot(i) + gamma1(i) * sig(estimation_error(i), alpha3) + mu1(i) * sig(estimation_error(i), beta3);
            imFeat_estimate(i) = imFeat_estimate(i) + x1_hat_dot(i)*step;   
                  
        }

        dist_linear << ibvs_dist(0), ibvs_dist(1),ibvs_dist(2);
        scaled_dist = Ryaw(quad_att(2)).transpose() * (quad_mass * z_des * dist_linear);

        im_feat_est_var.x = imFeat_estimate(0);
        im_feat_est_var.y = imFeat_estimate(1);
        im_feat_est_var.z = imFeat_estimate(2);
        im_feat_est_var.w = imFeat_estimate(3);

        im_feat_est_dot_var.x = imFeat_estimate_dot(0);
        im_feat_est_dot_var.y = imFeat_estimate_dot(1);
        im_feat_est_dot_var.z = imFeat_estimate_dot(2);
        im_feat_est_dot_var.w = imFeat_estimate_dot(3);

        im_feat_est_err_var.x = estimation_error(0);
        im_feat_est_err_var.y = estimation_error(1);
        im_feat_est_err_var.z = estimation_error(2);
        im_feat_est_err_var.w = estimation_error(3);
        
        ibvs_dist_var.x = ibvs_dist(0);
        ibvs_dist_var.y = ibvs_dist(1);
        ibvs_dist_var.z = ibvs_dist(2);
        ibvs_dist_var.w = ibvs_dist(3);

        scaled_ibvs_dist_var.x = scaled_dist(0);
        scaled_ibvs_dist_var.y = scaled_dist(1);
        scaled_ibvs_dist_var.z = scaled_dist(2);
        scaled_ibvs_dist_var.w = quad_mass * z_des * ibvs_dist(3);;

        im_feat_est_err_pub->publish(im_feat_est_err_var);
        im_feat_est_pub->publish(im_feat_est_var);
        im_feat_est_dot_pub->publish(im_feat_est_dot_var);
        dist_est_pub->publish(ibvs_dist_var);
        scaled_dist_pub->publish(scaled_ibvs_dist_var);

		loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}

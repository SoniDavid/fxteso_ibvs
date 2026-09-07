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
#include <std_msgs/msg/bool.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <math.h>
#include <vector>
//Including Eigen library
#include <eigen3/Eigen/Dense>
#include <stdexcept>

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
// The DESIRED servoing depth, thesis Eq. 5.81's z_d, which sets the input coefficient
// g_xi = -1/z_d on x/y/z. It has to track pos_ctrl's zD: the thesis' -2.5 m is its own
// choice of depth, not a constant. Overridden by the z_des parameter; see main().
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
    // Initialised, not left to fall through: NaN compares false against >0, <0 and ==0, so
    // an uninitialised `result` was returned for it - undefined behaviour that surfaced as
    // an arbitrary finite kick into the integrators rather than a detectable NaN.
    float result = 0;
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

// The no-lock output is a sentinel, not a measurement: integrating against it winds ibvs_dist on
// a fabricated error. Assumption 8 puts this outside the design, so hold rather than estimate.
bool imgFeat_valid = false;

void imFeatValidCallback(const std_msgs::msg::Bool::ConstSharedPtr v)
{
	imgFeat_valid = v->data;
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

    // The one gain that has to differ per plant: plant:=px4's inner loop lags 244 ms against
    // analytic's ~80 ms, so it needs a lower value here to get a truer error_dot out of x2_hat.
    const float gamma1_xy = node->declare_parameter<double>("gamma1_xy", 18.0);

    // Above zero, replaces the x/y triple with a triple pole at -observer_omega, i.e. thesis
    // Eq. 4.72's Hurwitz polynomial placed rather than hand-picked. Zero keeps gamma1_xy.
    const double observer_omega = node->declare_parameter<double>("observer_omega", 0.0);

    // The remaining departures from Table 5.3, exposed so the thesis set can be flown as-is:
    // thesis has gamma2_xy 10, gamma3_xy 7, gamma3_yaw 7.
    const float gamma2_xy = node->declare_parameter<double>("gamma2_xy", 20.0);
    const float gamma3_xy = node->declare_parameter<double>("gamma3_xy", 4.0);
    const float gamma3_yaw = node->declare_parameter<double>("gamma3_yaw", 3.0);
    // Yaw's position injection, 5 against x/y's 18. Low gamma1 measured worse on x/y, and the
    // yaw channel feeds a second integrator in pos_ctrl, so it is exposed to be swept.
    const float gamma1_yaw = node->declare_parameter<double>("gamma1_yaw", 5.0);
    const float gamma2_yaw = node->declare_parameter<double>("gamma2_yaw", 16.0);
    // Remark 5 tunes alpha and beta FIRST and calls alpha the noise-sensitivity knob; Eq. 4.72
    // wants alpha just under 1 and gamma4 > L1, measured ~0.05 on yaw against the flown 0.001.
    const float alpha_yaw = node->declare_parameter<double>("alpha_yaw", 0.75);
    const float beta_yaw = node->declare_parameter<double>("beta_yaw", 1.2);
    const float gamma4_yaw = node->declare_parameter<double>("gamma4_yaw", 0.001);
    // Yaw's g(xi)*u sign. Thesis Eq. 5.81 gives -1 and pos_ctrl's Omega carries -1, but this
    // channel has always been flown at +1. Default is as-flown so nothing changes silently.
    const float eso_yaw_sign = node->declare_parameter<double>("eso_yaw_sign", 1.0);

    // Added to the observer's initial state estimate, so this vector IS the seeded initial
    // estimation error. Zeros reproduce the thesis start; sweeping it is how the fixed-time
    // convergence claim - settling time independent of initial error - gets tested.
    const std::vector<double> x0_offset =
        node->declare_parameter<std::vector<double>>("initial_estimate_offset",
                                                     {0.0, 0.0, 0.0, 0.0});
    if (x0_offset.size() != 4)
        throw std::runtime_error("initial_estimate_offset needs exactly 4 values (qx,qy,qz,qpsi).");

    // Thesis Eq. 5.81: g_xi,x = g_xi,y = g_xi,z = -1/z_d, with z_d "the desired normal distance
    // between the UAV and its target". It is the same quantity as pos_ctrl's zD and must be set
    // to it - a mismatch leaves the part of the command the observer cannot explain to be booked
    // as disturbance, which pos_ctrl then feeds back in. Yaw is exempt: g_xi,psi = -1, no depth.
    z_des = node->declare_parameter<double>("z_des", z_des);

    //ROS publishers and subscribers
    auto im_feat_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("ImFeat_vector", 1, imFeatCallback);
    auto im_feat_valid_sub = node->create_subscription<std_msgs::msg::Bool>("ImFeat_valid", 1, imFeatValidCallback);
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

    imFeat_estimate << imFeat(0) + x0_offset[0], imFeat(1) + x0_offset[1],
                      1.7 + x0_offset[2], 0 + x0_offset[3];
    imFeat_estimate_dot << 0,0,0,0;
    ibvs_dist << 0, 0, 0, 0;

    estimation_error << imFeat(0), imFeat(1), 0, 0;
    x1_hat_dot << 0, 0, 0, 0;
    x2_hat_dot << 0, 0, 0, 0;
    x3_hat_dot << 0, 0, 0, 0;

    // //GAINS WITH MODEL UNCERTAINTIES
   // gamma2/gamma1 on x/y sets how much of x1_hat_dot reaches x2_hat - which pos_ctrl uses
   // as error_dot, its only damping term; at the thesis' 18/10 the injection took 75% of it.
   gamma1 << gamma1_xy, gamma1_xy, 16, gamma1_yaw;
   gamma2 << gamma2_xy, gamma2_xy, 14, gamma2_yaw;
   // Yaw is 3, not the thesis' 7; x/y are 4 for a related reason - ibvs_dist integrates this
   // gain and pos_ctrl feeds it back in phase, which was 0.5 of the command at the 0.3 Hz mode.
   gamma3 << gamma3_xy, gamma3_xy, 21, gamma3_yaw;
   gamma4 << 0.001, 0.001, 0.001, gamma4_yaw;

   if (observer_omega > 0.0) {
       const double w = observer_omega;
       gamma1(0) = gamma1(1) = 3 * w;
       gamma2(0) = gamma2(1) = 3 * w * w;
       gamma3(0) = gamma3(1) = w * w * w;
   }
  
     /*
     
    gamma1 << 16, 16, 16, 12;
   gamma2 << 14, 14, 14, 14;
   gamma3 << 6, 6, 21, 3;
   gamma4 << 0.0001, 0.0001, 0.001, 0.001;
*/


    
    mu1 = gamma1;
    mu2 = gamma2;
    mu3 = gamma3;

    // The node logged nothing at all until 2026-09-04, which is how a z_des frozen at 2.5 while
    // zD moved to 1.2 went unnoticed. A bag records neither, and run_matrix deletes a successful
    // run's launch log, so this line plus the manifest are the record of what was flown.
    RCLCPP_INFO(node->get_logger(),
                "servoing depth z_des = %.3f m, mass = %.3f kg%s", z_des, quad_mass,
                eso_yaw_sign < 0.0 ? ", yaw g(xi) sign -1 (thesis Eq. 5.81)" : "");
    RCLCPP_INFO(node->get_logger(),
                "gains xy (%.3g,%.3g,%.3g) z (%.3g,%.3g,%.3g) yaw (%.3g,%.3g,%.3g) "
                "gamma4_yaw %.3g alpha %.3g beta %.3g",
                gamma1(0), gamma2(0), gamma3(0), gamma1(2), gamma2(2), gamma3(2),
                gamma1(3), gamma2(3), gamma3(3), gamma4(3), alpha_yaw, beta_yaw);
    

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
        // No measurement -> do not step the observer. Every state is held, nothing is reset:
        // the equations are untouched, they simply are not run on a value that is not a
        // measurement. See imFeatValidCallback above.
        if (!imgFeat_valid)
        {
            loop_rate.sleep();   // SimRate spins the executor, so callbacks still run
            continue;
        }

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
                alpha = alpha_yaw;
                beta = beta_yaw;

                alpha2 = (alpha + 1)/2;
                beta2 = (beta + 1)/2;
    
                alpha3 = (alpha + 2)/3;
                beta3 = (beta + 2)/3;
                fx = 0;
                gx_u = eso_yaw_sign * ctrl_input(3);
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

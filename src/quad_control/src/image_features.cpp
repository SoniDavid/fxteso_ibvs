//Including ROS libraries
#include <rclcpp/rclcpp.hpp>
#include "quad_common/sim_rate.hpp"
#include <chrono>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
//Including C++ nominal libraries
#include <iostream>
#include <map>
#include <set>
#include <math.h>
#include <cmath>
#include <string>
#include <vector>
//Including opencv libraries
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
//Including Eigen library
#include <eigen3/Eigen/Dense>

//Declaring global variables
cv::Mat frame;

// float, not int: these feed mu11/mu20/mu02 directly, and truncating each centroid to a
// whole pixel puts a systematic ~-0.5 px bias on exactly the quantities the yaw feature
// is built from.
float ug, ug1, ug2, ug3, ug4;
float ng, ng1, ng2, ng3, ng4;
int p1,p2,p3,p4;
float p1x,p2x,p3x,p4x;
float p1y,p2y,p3y,p4y;

Eigen::Vector3f p1_vs_cf;
Eigen::Vector3f p2_vs_cf;
Eigen::Vector3f p3_vs_cf;
Eigen::Vector3f p4_vs_cf;

Eigen::Vector3f p1_vs_vf;
Eigen::Vector3f p2_vs_vf;
Eigen::Vector3f p3_vs_vf;
Eigen::Vector3f p4_vs_vf;

Eigen::Vector3f uav_att;

Eigen::Vector3f e3;

float beta_p1;
float beta_p2;
float beta_p3;
float beta_p4;

float ug_vs, ng_vs;
float mu20,mu02,mu11;
float den;

float qx,qy,qz,qpsi,qyaw;
// Branch state for the yaw feature. atan2 resolves 2*phi over a full turn, so qpsi is
// unambiguous over +-pi/2; qpsi_turns carries it beyond that, and is reset on lock loss.
float qpsi_prev = 0.0f;
float qpsi_turns = 0.0f;
bool qpsi_have_prev = false;
float a;
// Paired with the intrinsics: a scales as fx^2, so aD must move with fx or the depth setpoint
// moves instead. Overridden by launch; this literal is the default camera's value.
float aD = 0.00000076589;

//Camera intrinsic parameters
float focal_length = 0.00304;

// Intrinsics, taken from /quad/camera/camera_info once it arrives. Only the ratio
// pixel_size/focal_length = 1/fx enters the model, so focal_length stays a nominal scale and
// pixel_size is derived from fx. The literals below are the fallback until the topic is seen.
bool have_caminfo = false;
float cam_fx = 693.3f;      // 0.00304 / 4.38462e-6, the value the constants below imply
float cam_cx = 410.0f;      // 820 / 2
float cam_cy = 308.0f;      // 616 / 2
cv::Mat cam_K, cam_D;
bool cam_distorted = false; // false keeps the undistortion out of the path entirely

void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    cam_fx = info->k[0];
    cam_cx = info->k[2];
    cam_cy = info->k[5];
    cam_K = (cv::Mat_<double>(3, 3) << info->k[0], info->k[1], info->k[2],
                                       info->k[3], info->k[4], info->k[5],
                                       info->k[6], info->k[7], info->k[8]);
    cam_D = cv::Mat(1, static_cast<int>(info->d.size()), CV_64F);
    cam_distorted = false;
    for (size_t i = 0; i < info->d.size(); ++i)
    {
        cam_D.at<double>(0, static_cast<int>(i)) = info->d[i];
        if (info->d[i] != 0.0)
            cam_distorted = true;
    }
    if (!have_caminfo)
        RCLCPP_INFO(rclcpp::get_logger("image_features"),
                    "camera_info: %ux%u, fx=%.1f cx=%.1f cy=%.1f, %s",
                    info->width, info->height, cam_fx, cam_cx, cam_cy,
                    cam_distorted ? "distortion modelled" : "no distortion");
    have_caminfo = true;
}
// Tracks the camera in quad_description/models/F450/model.sdf - change one, change both.
float pixel_size = 0.00000438462;

/////////////////////////Functions///////////////////////////////

//Matrix R_phi_theta
Eigen::Matrix3f Rtp(float roll, float pitch)
{
    Eigen::Matrix3f pitch_mat;
    pitch_mat << cos(pitch), 0.0, sin(pitch),
        0.0, 1.0, 0.0,
        -sin(pitch), 0.0, cos(pitch);


    Eigen::Matrix3f roll_mat;
    roll_mat << 1.0, 0.0, 0.0,
        0.0, cos(roll), -sin(roll),
        0.0, sin(roll), cos(roll);

    Eigen::Matrix3f R_tp;
    R_tp = pitch_mat * roll_mat;

    return R_tp;

}

Eigen::Matrix3f Ryaw(float yaw)
{   
    Eigen::Matrix3f yaw_mat;
    yaw_mat << cos(yaw), -sin(yaw),0,
                sin(yaw), cos(yaw), 0,
                0, 0, 1;
    return yaw_mat;
}

/////////////////////ROS Subscribers//////////////////////////////////
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
	try
	{
		frame = cv_bridge::toCvShare(msg, "bgr8")->image; //Saving the image
	}
	
	catch(cv_bridge::Exception& e)
	{ 
		RCLCPP_ERROR(rclcpp::get_logger("image_features"), "Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

/*void attitude_callback(const geometry_msgs::msg::Vector3::ConstSharedPtr att)
{
	uav_att(0) = att->x;
	uav_att(1) = att->y;
	uav_att(2) = att->z;
}*/

void attEstCallback(const geometry_msgs::msg::Twist::ConstSharedPtr aE)
{
    uav_att(0) = aE->linear.x;
    uav_att(1) = aE->linear.y;
    uav_att(2) = aE->linear.z;
    
    
}


////////////////////Main program//////////////////////////////////////
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("image_features");

	fxteso::SimRate loop_rate(node, 50);	
    
    //ROS publishers and subscribers
	auto im_feat_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("ImFeat_vector",100);
	// True only when all four markers decoded. ImFeat_vector's (0,0,1,0) no-lock sentinel is
	// numerically identical to the servo setpoint, so telling the two apart requires this.
	auto im_feat_valid_pub = node->create_publisher<std_msgs::msg::Bool>("ImFeat_valid",100);
    auto a_value_pub = node->create_publisher<std_msgs::msg::Float64>("a_value",100);
	auto u_coord_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("u_coordinates",100);
	auto n_coord_pub = node->create_publisher<geometry_msgs::msg::Quaternion>("n_coordinates",100);
	
    auto sub = node->create_subscription<sensor_msgs::msg::Image>("/quad/camera/image_raw", 1, imageCallback); //Gazebo_camera 
    //auto sub = node->create_subscription<sensor_msgs::msg::Image>("camera/image", 1, imageCallback); //Real camera
	//auto attitude_sub = node->create_subscription<geometry_msgs::msg::Vector3>("quad_attitude", 1, attitude_callback);
	auto attitude_sub = node->create_subscription<geometry_msgs::msg::Twist>("attitude_estimates", 1, attEstCallback);
	auto caminfo_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
		"/quad/camera/camera_info", 1, cameraInfoCallback);

	// The image-moment area at the desired depth. It scales as fx^2, so a different
	// camera needs a different value - hover at zD and read /a_value to measure it.
	aD = node->declare_parameter<double>("aD", aD);

	// Intrinsics from the camera the world is actually rendering. gz-sim does not publish
	// camera_info here, so these come from the launch file, which is the same place the SDF
	// camera block is chosen - one source of truth. camera_info still wins if it ever arrives.
	const double p_hfov = node->declare_parameter<double>("camera_hfov", 1.085595);
	const int p_width = node->declare_parameter<int>("camera_width", 820);
	const int p_height = node->declare_parameter<int>("camera_height", 616);
	// Brown-Conrady k1,k2,p1,p2,k3. All-zero means no distortion and the undistortion step
	// is skipped entirely; an empty default cannot be statically typed, hence the zeros.
	const std::vector<double> p_dist = node->declare_parameter<std::vector<double>>(
		"camera_distortion", {0.0, 0.0, 0.0, 0.0, 0.0});
	cam_fx = p_width / (2.0 * tan(p_hfov / 2.0));
	cam_cx = p_width / 2.0f;
	cam_cy = p_height / 2.0f;
	cam_K = (cv::Mat_<double>(3, 3) << cam_fx, 0, cam_cx, 0, cam_fx, cam_cy, 0, 0, 1);
	if (!p_dist.empty())
	{
		cam_D = cv::Mat(1, static_cast<int>(p_dist.size()), CV_64F);
		for (size_t i = 0; i < p_dist.size(); ++i)
		{
			cam_D.at<double>(0, static_cast<int>(i)) = p_dist[i];
			if (p_dist[i] != 0.0)
				cam_distorted = true;
		}
	}
	RCLCPP_INFO(node->get_logger(), "camera: %dx%d, hFOV %.4f rad, fx %.1f px, %s",
	            p_width, p_height, p_hfov, cam_fx,
	            cam_distorted ? "distortion modelled" : "no distortion");
	
	    
    //Declaring local variables
    geometry_msgs::msg::Quaternion im_feat_vec;
    std_msgs::msg::Bool im_feat_valid;
    std_msgs::msg::Float64 a_val;
	geometry_msgs::msg::Quaternion u_cam_coord;
	geometry_msgs::msg::Quaternion n_cam_coord;

    e3 << 0,0,1;
    
	//Loading the dictionary where the aruco markers belong to. 7x7 is the thesis' and every
	// recorded bag's; 4x4 is 6 modules a side against 9, so it decodes at ~2/3 the pixel size.
	const std::map<std::string, int> kDicts = {
		{"4x4", cv::aruco::DICT_4X4_50}, {"5x5", cv::aruco::DICT_5X5_50},
		{"6x6", cv::aruco::DICT_6X6_50}, {"7x7", cv::aruco::DICT_7X7_50}};
	const std::string dict_name = node->declare_parameter<std::string>("marker_dict", "7x7");
	const auto dict_it = kDicts.find(dict_name);
	if (dict_it == kDicts.end())
	{
		RCLCPP_FATAL(node->get_logger(),
		             "marker_dict:=%s is not one of 4x4, 5x5, 6x6, 7x7.", dict_name.c_str());
		return 1;
	}
	RCLCPP_INFO(node->get_logger(), "marker dictionary: DICT_%s_50",
	            dict_name == "4x4" ? "4X4" : dict_name == "5x5" ? "5X5" :
	            dict_name == "6x6" ? "6X6" : "7X7");
	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dict_it->second);

	qx = 0;
	qy = 0;
	qz = 1;
	qpsi  = 0;
	//Publishing data via Rostopics
    im_feat_vec.x = qx;
    im_feat_vec.y = qy;
	im_feat_vec.z = qz;
    im_feat_vec.w = qpsi;

	im_feat_valid.data = false;
	im_feat_pub->publish(im_feat_vec);
	im_feat_valid_pub->publish(im_feat_valid);
	loop_rate.sleepFor(1.7);

    while (rclcpp::ok())
	{
        //Initializing the detector parameters using default values
		cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
		//Declaring the 2D vectors that contain the aruco's corners and rejected candidates 
		std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
		//Declaring a vector to save de ID numbers of the detected arucos
		std::vector<int> markerIds;
		
		if(!frame.empty())
		{
			//Detect the markers in the image
		cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectCandidates);
		}
		else
		{
			std::cout << "empty " << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;
			qpsi_have_prev = false;
			qpsi_turns = 0.0f;
			 //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_valid.data = false;
			im_feat_pub->publish(im_feat_vec);
			im_feat_valid_pub->publish(im_feat_valid);
		}

		// Identity, not just count: the per-point assignment below keys on the IDs, and a
		// wrong or duplicated one falls through every branch leaving the PREVIOUS frame's
		// centroids in place - a corrupt reading that still looks like a lock.
		const bool ids_ok = (markerIds.size() == 4) &&
			(std::set<int>(markerIds.begin(), markerIds.end()) == std::set<int>{4, 6, 8, 10});

        //If no arucos are detected, then print on screen
		if (!ids_ok)
		{
			if (markerIds.size() != 4)
				std::cout << "I see " << markerIds.size() << " arucos only. I need 4 to work properly" << std::endl;
			else
				std::cout << "I see 4 arucos but not {4,6,8,10}" << std::endl;
			qx = 0;
			qy = 0;
			qz = 1;
			qpsi  = 0;
			qpsi_have_prev = false;
			qpsi_turns = 0.0f;

			 //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			im_feat_valid.data = false;
			im_feat_pub->publish(im_feat_vec);
			im_feat_valid_pub->publish(im_feat_valid);
		}

		else
        {
            //Assignment of Point 1
			if (markerIds[0] == 6)
			{
				//Corner extraction
				cv::Point2f p11 = markerCorners[0].at(0);
				cv::Point2f p21 = markerCorners[0].at(1);
				cv::Point2f p31 = markerCorners[0].at(2);
				cv::Point2f p41 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4.0f;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4.0f;
			}
			else if (markerIds[1] == 6)
			{
				//Corner extraction
				cv::Point2f p11 = markerCorners[1].at(0);
				cv::Point2f p21 = markerCorners[1].at(1);
				cv::Point2f p31 = markerCorners[1].at(2);
				cv::Point2f p41 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4.0f;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4.0f;
			}
			else if (markerIds[2] == 6)
			{
				//Corner extraction
				cv::Point2f p11 = markerCorners[2].at(0);
				cv::Point2f p21 = markerCorners[2].at(1);
				cv::Point2f p31 = markerCorners[2].at(2);
				cv::Point2f p41 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4.0f;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4.0f;
			}
			else if (markerIds[3] == 6)
			{
				//Corner extraction
				cv::Point2f p11 = markerCorners[3].at(0);
				cv::Point2f p21 = markerCorners[3].at(1);
				cv::Point2f p31 = markerCorners[3].at(2);
				cv::Point2f p41 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug1 = (p11.x + p21.x + p31.x + p41.x) / 4.0f;
				ng1 = (p11.y + p21.y + p31.y + p41.y) / 4.0f;
			}
			///////////////////////////////////////////////////////////////////////////////////////////////////////////
			//Assignment of Point 2
			if (markerIds[0] == 4)
			{
				//Corner extraction
				cv::Point2f p12 = markerCorners[0].at(0);
				cv::Point2f p22 = markerCorners[0].at(1);
				cv::Point2f p32 = markerCorners[0].at(2);
				cv::Point2f p42 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4.0f;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4.0f;
			}
			else if (markerIds[1] == 4)
			{
				//Corner extraction
				cv::Point2f p12 = markerCorners[1].at(0);
				cv::Point2f p22 = markerCorners[1].at(1);
				cv::Point2f p32 = markerCorners[1].at(2);
				cv::Point2f p42 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4.0f;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4.0f;
			}
			else if (markerIds[2] == 4)
			{
				//Corner extraction
				cv::Point2f p12 = markerCorners[2].at(0);
				cv::Point2f p22 = markerCorners[2].at(1);
				cv::Point2f p32 = markerCorners[2].at(2);
				cv::Point2f p42 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4.0f;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4.0f;
			}
			else if (markerIds[3] == 4)
			{
				//Corner extraction
                cv::Point2f p12 = markerCorners[3].at(0);
				cv::Point2f p22 = markerCorners[3].at(1);
				cv::Point2f p32 = markerCorners[3].at(2);
				cv::Point2f p42 = markerCorners[3].at(3);

                //Centroid of the aruco
				ug2 = (p12.x + p22.x + p32.x + p42.x) / 4.0f;
				ng2 = (p12.y + p22.y + p32.y + p42.y) / 4.0f;
			}
			///////////////////////////////////////////////////////////////////////////////77
			//Assignment of Point 3
			if (markerIds[0] == 8)
			{
				//Corner extraction
				cv::Point2f p13 = markerCorners[0].at(0);
				cv::Point2f p23 = markerCorners[0].at(1);
				cv::Point2f p33 = markerCorners[0].at(2);
				cv::Point2f p43 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4.0f;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4.0f;
			}
			else if (markerIds[1] == 8)
			{
				//Corner extraction
				cv::Point2f p13 = markerCorners[1].at(0);
				cv::Point2f p23 = markerCorners[1].at(1);
				cv::Point2f p33 = markerCorners[1].at(2);
				cv::Point2f p43 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4.0f;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4.0f;
			}
			else if (markerIds[2] == 8)
			{
				//Corner extraction
				cv::Point2f p13 = markerCorners[2].at(0);
				cv::Point2f p23 = markerCorners[2].at(1);
				cv::Point2f p33 = markerCorners[2].at(2);
				cv::Point2f p43 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4.0f;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4.0f;
			}
			else if (markerIds[3]==8)
			{
				//Corner extraction
				cv::Point2f p13 = markerCorners[3].at(0);
				cv::Point2f p23 = markerCorners[3].at(1);
				cv::Point2f p33 = markerCorners[3].at(2);
				cv::Point2f p43 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug3 = (p13.x + p23.x + p33.x + p43.x) / 4.0f;
				ng3 = (p13.y + p23.y + p33.y + p43.y) / 4.0f;
			}
		////////////////////////////////////////////////////////////////////////////////////7
			//Assignment of Point 4
			if (markerIds[0] == 10)
			{
				//Corner extraction
        		cv::Point2f p14 = markerCorners[0].at(0);
				cv::Point2f p24 = markerCorners[0].at(1);
				cv::Point2f p34 = markerCorners[0].at(2);
				cv::Point2f p44 = markerCorners[0].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4.0f;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4.0f;
			}
			else if (markerIds[1] == 10)
			{
				//Corner extraction
				cv::Point2f p14 = markerCorners[1].at(0);
				cv::Point2f p24 = markerCorners[1].at(1);
				cv::Point2f p34 = markerCorners[1].at(2);
				cv::Point2f p44 = markerCorners[1].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4.0f;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4.0f;
			}
			else if (markerIds[2] == 10)
			{
				//Corner extraction
				cv::Point2f p14 = markerCorners[2].at(0);
				cv::Point2f p24 = markerCorners[2].at(1);
				cv::Point2f p34 = markerCorners[2].at(2);
				cv::Point2f p44 = markerCorners[2].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4.0f;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4.0f;
			}
			else if (markerIds[3] == 10)
			{
				//Corner extraction
				cv::Point2f p14 = markerCorners[3].at(0);
				cv::Point2f p24 = markerCorners[3].at(1);
				cv::Point2f p34 = markerCorners[3].at(2);
				cv::Point2f p44 = markerCorners[3].at(3);

				//Centroid of the aruco
				ug4 = (p14.x + p24.x + p34.x + p44.x) / 4.0f;
				ng4 = (p14.y + p24.y + p34.y + p44.y) / 4.0f;
			}
			
            //Centroid of the target
			ug = (ug1 + ug2 + ug3 + ug4) / 4;
			ng = (ng1 + ng2 + ng3 + ng4) / 4;
			
            /* Indicators
			cv::circle(frame, cv::Point(ug, ng), 8, cv::Scalar(255, 0, 255));
			cv::circle(frame, cv::Point(ug1, ng1), 8, cv::Scalar(255, 0, 0));
			cv::circle(frame, cv::Point(ug2, ng2), 8, cv::Scalar(0, 255, 0));
			cv::circle(frame, cv::Point(ug3, ng3), 8, cv::Scalar(0, 0, 255));
			cv::circle(frame, cv::Point(ug4, ng4), 8, cv::Scalar(255, 255, 0));
			*/
			

            //Declaring the points required for visual servoing
			cv::Point2f p1 = cv::Point2f(ug1,ng1);
			cv::Point2f p2 = cv::Point2f(ug2,ng2);
			cv::Point2f p3 = cv::Point2f(ug3,ng3);
			cv::Point2f p4 = cv::Point2f(ug4,ng4);

            //Changing Image coordinates system from the left-top to the center.
			/*
											+y	^		
												|
												|
												|
			      					-x <--------|---------> +x
					      						|
			      								|
			      								|		
			      								|
			      								-y		
					
			*/
			// Lens distortion is undone on the four centroids, the way it would be on
			// hardware. Skipped when the coefficients are zero, so the pinhole path is
			// bit-for-bit what it was.
			std::vector<cv::Point2f> centroids = {
				cv::Point2f((float)p1.x, (float)p1.y), cv::Point2f((float)p2.x, (float)p2.y),
				cv::Point2f((float)p3.x, (float)p3.y), cv::Point2f((float)p4.x, (float)p4.y)};
			if (cam_distorted)
				cv::undistortPoints(centroids, centroids, cam_K, cam_D, cv::noArray(), cam_K);

			p1x = centroids[0].x-cam_cx;
			p1y = -(centroids[0].y-cam_cy);
			
			p2x = centroids[1].x-cam_cx;
			p2y = -(centroids[1].y-cam_cy);
			
			p3x = centroids[2].x-cam_cx;
			p3y = -(centroids[2].y-cam_cy);
			
			p4x = centroids[3].x-cam_cx;
			p4y = -(centroids[3].y-cam_cy);

			// Only pixel_size/focal_length = 1/fx enters the model below, so deriving
			// pixel_size from the reported fx keeps it consistent with the camera actually
			// rendering. The old hardcoded pair disagreed with the SDF's FOV by 2%.
			pixel_size = focal_length / cam_fx;

            //Camera frame point data (u,n,focal_length)
            p1_vs_cf << p1x*pixel_size, p1y*pixel_size, focal_length;
			p2_vs_cf << p2x*pixel_size, p2y*pixel_size, focal_length;
			p3_vs_cf << p3x*pixel_size, p3y*pixel_size, focal_length;
			p4_vs_cf << p4x*pixel_size, p4y*pixel_size, focal_length;


            //Camera frame to Virtual frame conversion
			beta_p1 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p1_vs_cf);
			beta_p2 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p2_vs_cf);
			beta_p3 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p3_vs_cf);
			beta_p4 = focal_length/(e3.transpose()*Rtp(uav_att(0),uav_att(1))*p4_vs_cf);

            p1_vs_vf = beta_p1*Rtp(uav_att(0),uav_att(1))*p1_vs_cf;
			p2_vs_vf = beta_p2*Rtp(uav_att(0),uav_att(1))*p2_vs_cf;
			p3_vs_vf = beta_p3*Rtp(uav_att(0),uav_att(1))*p3_vs_cf;
			p4_vs_vf = beta_p4*Rtp(uav_att(0),uav_att(1))*p4_vs_cf;

			/*
			p1_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p1_vs_vf;
			p2_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p2_vs_vf;
			p3_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p3_vs_vf;
			p4_vs_vf = Ryaw(uav_att(2)).transpose()*Ryaw(uav_att(2)).transpose()*p4_vs_vf;
			*/	
            //Ordinary moments. Centroid
			ug_vs = (p1_vs_vf(0) + p2_vs_vf(0) + p3_vs_vf(0) + p4_vs_vf(0)) / 4;
			ng_vs = (p1_vs_vf(1) + p2_vs_vf(1) + p3_vs_vf(1) + p4_vs_vf(1)) / 4;
			
			//Momentos centrados
			mu20 = powf((p1_vs_vf(0) - ug_vs), 2) +  powf((p2_vs_vf(0) - ug_vs), 2) +  powf((p3_vs_vf(0) - ug_vs), 2) +  powf((p4_vs_vf(0) - ug_vs), 2);
			mu02 = powf((p1_vs_vf(1) - ng_vs), 2) +  powf((p2_vs_vf(1) - ng_vs), 2) +  powf((p3_vs_vf(1) - ng_vs), 2) +  powf((p4_vs_vf(1) - ng_vs), 2);
			mu11 = ((p1_vs_vf(0) - ug_vs) * (p1_vs_vf(1) - ng_vs)) + ((p2_vs_vf(0) - ug_vs) * (p2_vs_vf(1) - ng_vs)) + ((p3_vs_vf(0) - ug_vs) * (p3_vs_vf(1) - ng_vs)) + ((p4_vs_vf(0) - ug_vs) * (p4_vs_vf(1) - ng_vs));
			den = mu20-mu02;

            //Image features vector
			a = mu20 + mu02;
			qz = sqrt(aD/a);
			qx = qz * ng_vs/focal_length;
			qy = qz * ug_vs/focal_length;
			// atan2, not Eq. 5.8's single-argument atan: that confines the feature to +-pi/4 and
			// jumps by pi/2 at 45 deg of relative yaw, inverting the yaw feedback.
			float qpsi_raw = -0.5 * atan2(2*mu11, den);
			// Both the moment sign and atan2's wrap quantise qpsi in pi/2, so snap every
			// discontinuity to the nearest multiple of it. The seed matters: without it a plate
			// with b > a servos about +-pi/2 rather than zero.
			if (!qpsi_have_prev)
			{
				qpsi_turns = -M_PI_2 * std::round(qpsi_raw / M_PI_2);
			}
			else
			{
				float d = qpsi_raw + qpsi_turns - qpsi_prev;
				qpsi_turns -= M_PI_2 * std::round(d / M_PI_2);
			}
			qpsi = qpsi_raw + qpsi_turns;
			qpsi_prev = qpsi;
			qpsi_have_prev = true;

            //Publishing data via Rostopics
            im_feat_vec.x = qx;
            im_feat_vec.y = qy;
            im_feat_vec.z = qz;
            im_feat_vec.w = qpsi;

			u_cam_coord.x = p1_vs_vf(0)/pixel_size;
			u_cam_coord.y = p2_vs_vf(0)/pixel_size;
			u_cam_coord.z = p3_vs_vf(0)/pixel_size;
			u_cam_coord.w = p4_vs_vf(0)/pixel_size;

			n_cam_coord.x = p1_vs_vf(1)/pixel_size;
			n_cam_coord.y = p2_vs_vf(1)/pixel_size;
			n_cam_coord.z = p3_vs_vf(1)/pixel_size;
			n_cam_coord.w = p4_vs_vf(1)/pixel_size;

            a_val.data = a;			

			im_feat_valid.data = true;
           	im_feat_pub->publish(im_feat_vec);
			im_feat_valid_pub->publish(im_feat_valid);
            a_value_pub->publish(a_val);
			u_coord_pub->publish(u_cam_coord);
			n_coord_pub->publish(n_cam_coord);
			
			/*
			std::cout << "p1 " << p1_vs_vf(0) << ", " << p1_vs_vf(1) << std::endl;
			std::cout << "p2 " << p2_vs_vf(0) << ", " << p2_vs_vf(1) << std::endl;
			std::cout << "p3 " << p3_vs_vf(0) << ", " << p3_vs_vf(1) << std::endl;
			std::cout << "p4 " << p4_vs_vf(0) << ", " << p4_vs_vf(1) << std::endl;
			std::cout << "mu11 " << mu11 << std::endl;*/
			std::cout << "OK" << std::endl;
			
        }

		loop_rate.sleep();
	}  

    rclcpp::shutdown();

    return 0;
}

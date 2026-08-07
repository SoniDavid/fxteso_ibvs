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
#include <fstream>
#include <string>
#include <sstream>
//Including Eigen library
#include <eigen3/Eigen/Dense>

using namespace std;

float dist_x;
float dist_y;
float dist_z;
int i = 0;
float step = 0.01;
float t = 0;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("disturbances");

	fxteso::SimRate loop_rate(node, 100);
    
    //ROS publishers and subscribers
    auto disturbances_pub = node->create_publisher<geometry_msgs::msg::Vector3>("disturbances",100);
    geometry_msgs::msg::Vector3 disturbances_var;

    ifstream myFile;
    string line;
    myFile.open("/home/armando/Documents/visual_servoing_ws/src/fxteso_asgibvs/src/csv_files/dist3.csv");
    getline(myFile, line);
    	
    disturbances_var.x = 0;
    disturbances_var.y = 0;
    disturbances_var.z = 0;
    disturbances_pub->publish(disturbances_var);
    loop_rate.sleepFor(8.0);

   while (getline(myFile, line) && rclcpp::ok())
    {       
        stringstream stream(line); // Convertir la cadena a un stream
        string x, y, z;
        // Extraer todos los valores de esa fila
        getline(stream, x, ',');
        getline(stream, y, ',');
        getline(stream, z, ',');

        dist_x = stof(x);
        dist_y = stof(y);
        dist_z = stof(z);

        disturbances_var.x = dist_x;
        disturbances_var.y = dist_y;
        disturbances_var.z = dist_z;

        disturbances_pub->publish(disturbances_var);

		loop_rate.sleep();        
    }

    myFile.close();
   
    while (rclcpp::ok())
    {
    /*
        t = i*step;

        if (t >= 22 && t <= 25)
        {
             disturbances_var.x = 0.3;
             disturbances_var.y = 0.3;
             disturbances_var.z = 0.3;
        }
        else
        {
             disturbances_var.x = 0;
             disturbances_var.y = 0;
             disturbances_var.z = 0;
        }
        
        i++;
*/
  		disturbances_var.x = 0;
        disturbances_var.y = 0;
        disturbances_var.z = 0;

        disturbances_pub->publish(disturbances_var);
		loop_rate.sleep(); 
    }

   
    rclcpp::shutdown();

   
    return 0;
}


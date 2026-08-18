cmake_minimum_required(VERSION 3.8)
project(quad_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quad_common REQUIRED)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_compile_features(tf_broadcaster PUBLIC cxx_std_17)
ament_target_dependencies(tf_broadcaster
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros quad_common)

install(TARGETS tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# foxglove/ carries the Studio layout and its checker; config/bag_topics.yaml is the
# recording list sim.launch.py reads. analysis/ are offline bag tools - run them from the
# source tree, they take bag paths and need no ROS graph.
install(DIRECTORY launch config foxglove analysis
  DESTINATION share/${PROJECT_NAME})

ament_package()

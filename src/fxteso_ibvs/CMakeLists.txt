cmake_minimum_required(VERSION 3.8)
project(fxteso_ibvs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(Eigen3 REQUIRED)
# System OpenCV is 4.5.4, which still ships the pre-4.7 cv::aruco API
# (Ptr<Dictionary>, DetectorParameters::create(), free detectMarkers), so
# image_features.cpp compiles unchanged from the ROS 1 original.
find_package(OpenCV REQUIRED)

# include/fxteso_ibvs/sim_rate.hpp - the sim-time loop pacing every node uses. Header
# only, so it is an include path rather than a library.
include_directories(include)

# --- Algorithm nodes: straight rclcpp ports of fxteso_asgibvs -----------------
# Same gains, same rates, same topic names. Only the ROS plumbing differs.
set(IBVS_NODES
  uav_dynamics
  disturbances
  target_position
  att_ctrl
  fixed_eso
  td_attitude
  td_attitude_desired
  td_linear
)

foreach(node ${IBVS_NODES})
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  target_include_directories(${node} PUBLIC ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(${node} rclcpp std_msgs geometry_msgs sensor_msgs)
endforeach()

# pos_ctrl keeps its original filename so it diffs cleanly against the ROS 1 tree.
add_executable(pos_ctrl src/aibvs_pos_ctrl.cpp)
target_compile_features(pos_ctrl PUBLIC cxx_std_17)
target_include_directories(pos_ctrl PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(pos_ctrl rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(image_features src/image_features.cpp)
target_compile_features(image_features PUBLIC cxx_std_17)
target_include_directories(image_features PUBLIC ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_features ${OpenCV_LIBRARIES})
ament_target_dependencies(image_features rclcpp std_msgs geometry_msgs sensor_msgs cv_bridge)

# --- Gazebo Harmonic pose broadcaster ----------------------------------------
# Talks gz-transport directly: set_pose_vector has no ROS message equivalent.
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(gz_pose_broadcaster src/gz_pose_broadcaster.cpp)
target_compile_features(gz_pose_broadcaster PUBLIC cxx_std_17)
target_link_libraries(gz_pose_broadcaster
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
ament_target_dependencies(gz_pose_broadcaster rclcpp std_msgs geometry_msgs tf2)

# --- Foxglove visualisation ---------------------------------------------------
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_compile_features(tf_broadcaster PUBLIC cxx_std_17)
ament_target_dependencies(tf_broadcaster
  rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_ros)

install(TARGETS ${IBVS_NODES} pos_ctrl image_features gz_pose_broadcaster tf_broadcaster
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config models worlds foxglove
  DESTINATION share/${PROJECT_NAME})

ament_package()

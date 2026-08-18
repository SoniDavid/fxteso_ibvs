cmake_minimum_required(VERSION 3.8)
project(quad_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# quad_common/sim_rate.hpp - the sim-time loop pacing every node uses.
find_package(quad_common REQUIRED)

# --- Estimation and control nodes: straight rclcpp ports of fxteso_asgibvs -----
# Same gains, same rates, same topic names. Only the ROS plumbing differs.
set(IBVS_NODES
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
  ament_target_dependencies(${node} rclcpp std_msgs geometry_msgs sensor_msgs quad_common)
endforeach()

# pos_ctrl keeps its original filename so it diffs cleanly against the ROS 1 tree.
add_executable(pos_ctrl src/aibvs_pos_ctrl.cpp)
target_compile_features(pos_ctrl PUBLIC cxx_std_17)
target_include_directories(pos_ctrl PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(pos_ctrl rclcpp std_msgs geometry_msgs sensor_msgs quad_common)

# Gates the controllers on the plant, the target and a real marker lock. See the file
# header for why a wall-clock launch delay could not do this job.
add_executable(ibvs_gate src/ibvs_gate.cpp)
target_compile_features(ibvs_gate PUBLIC cxx_std_17)
ament_target_dependencies(ibvs_gate rclcpp geometry_msgs)

# OpenCV 4.6.0 on Noble still ships the pre-4.7 cv::aruco API, so image_features.cpp
# compiles unchanged from the ROS 1 original.
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(image_features src/image_features.cpp)
target_compile_features(image_features PUBLIC cxx_std_17)
target_include_directories(image_features PUBLIC ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(image_features ${OpenCV_LIBRARIES})
ament_target_dependencies(image_features rclcpp std_msgs geometry_msgs sensor_msgs cv_bridge quad_common)

install(TARGETS ${IBVS_NODES} pos_ctrl image_features ibvs_gate
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

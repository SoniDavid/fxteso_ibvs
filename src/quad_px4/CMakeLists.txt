cmake_minimum_required(VERSION 3.8)
project(quad_px4)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(quad_common REQUIRED)

# px4_state_adapter is the PX4 counterpart of quad_gz_sim's gz_state_adapter and
# uav_dynamics: same five state topics out, EKF2 in. px4_offboard_bridge is the other end -
# it turns pos_ctrl's desired attitude and thrust into OFFBOARD setpoints.
set(PX4_NODES
  px4_state_adapter
  px4_offboard_bridge
  px4_takeoff_gate
)

foreach(node ${PX4_NODES})
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  # include/quad_px4/px4_topic.hpp derives the _vN topic suffix from px4_msgs.
  target_include_directories(${node} PRIVATE include)
  ament_target_dependencies(${node}
    rclcpp std_msgs geometry_msgs px4_msgs tf2 quad_common)
endforeach()

install(TARGETS ${PX4_NODES}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# The airframe is consumed by PX4 through a symlink, not by ROS; installed so the path is
# discoverable from an installed workspace as well as the source tree.
install(DIRECTORY px4
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/link_px4_airframe.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(quad_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(quad_common REQUIRED)

# --- Plant and scenario nodes -------------------------------------------------
# uav_dynamics is the analytic plant (plant:=analytic); the scenario nodes drive it.
set(SIM_NODES
  uav_dynamics
  disturbances
  target_position
)

foreach(node ${SIM_NODES})
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  target_include_directories(${node} PUBLIC ${EIGEN3_INCLUDE_DIRS})
  ament_target_dependencies(${node} rclcpp std_msgs geometry_msgs sensor_msgs quad_common)
endforeach()

# The Gazebo-physics counterpart of uav_dynamics: same five output topics, state sourced
# from the BodyWrench plugin instead of a ROS-side integrator.
add_executable(gz_state_adapter src/gz_state_adapter.cpp)
target_compile_features(gz_state_adapter PUBLIC cxx_std_17)
ament_target_dependencies(gz_state_adapter rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

# --- Gazebo Harmonic plumbing -------------------------------------------------
# Talks gz-transport directly: set_pose_vector has no ROS message equivalent.
# On Jazzy, Harmonic comes from ros_gz's vendor packages, not a system gz install.
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# The Gazebo-physics plant. A gz-sim system plugin, not a ROS node: it has to write the
# wrench component inside the 1 ms physics step, which a 100 Hz ROS node cannot do.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

add_library(BodyWrenchSystem SHARED src/plugins/body_wrench_system.cc)
target_compile_features(BodyWrenchSystem PUBLIC cxx_std_17)
target_link_libraries(BodyWrenchSystem
  gz-sim::core
  gz-plugin::core
  gz-transport::core
  gz-msgs::core)

add_executable(gz_pose_broadcaster src/gz_pose_broadcaster.cpp)
target_compile_features(gz_pose_broadcaster PUBLIC cxx_std_17)
target_link_libraries(gz_pose_broadcaster
  gz-transport::core
  gz-msgs::core)
ament_target_dependencies(gz_pose_broadcaster rclcpp std_msgs geometry_msgs tf2 quad_common)

install(TARGETS ${SIM_NODES} gz_state_adapter gz_pose_broadcaster
  DESTINATION lib/${PROJECT_NAME})

# Plain lib/, not lib/${PROJECT_NAME}: gz_sim.launch.py points
# GZ_SIM_SYSTEM_PLUGIN_PATH at this package's prefix/lib.
install(TARGETS BodyWrenchSystem
  DESTINATION lib)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()

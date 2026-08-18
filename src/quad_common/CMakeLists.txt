cmake_minimum_required(VERSION 3.8)
project(quad_common)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Header only: no sources to compile, so an INTERFACE target is the whole library.
add_library(quad_common INTERFACE)
target_include_directories(quad_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(quad_common INTERFACE ${rclcpp_TARGETS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS quad_common EXPORT quad_commonTargets)

ament_export_targets(quad_commonTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

ament_package()

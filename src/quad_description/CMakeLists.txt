cmake_minimum_required(VERSION 3.8)
project(quad_description)

find_package(ament_cmake REQUIRED)

# Assets only. F450_base/model.sdf owns the camera block and the flight sensors PX4 looks up by
# name; F450 and F450_px4 merge-include it and differ only in actuation.
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_package()

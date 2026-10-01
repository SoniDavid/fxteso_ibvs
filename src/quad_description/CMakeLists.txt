cmake_minimum_required(VERSION 3.8)
project(quad_description)

find_package(ament_cmake REQUIRED)

# Assets only. F450_base/model.sdf owns the camera block and the flight sensors PX4 looks up by
# name; F450 and F450_px4 merge-include it and differ only in actuation.
# config/cameras.yaml + launch/camera_presets.py: the camera preset table sim and hardware share.
install(DIRECTORY models config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()

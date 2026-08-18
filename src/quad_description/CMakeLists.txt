cmake_minimum_required(VERSION 3.8)
project(quad_description)

find_package(ament_cmake REQUIRED)

# Assets only. models/F450_base/model.sdf sets the camera resolution and hfov that
# image_features.cpp hardcodes its focal length and pixel size against, and carries the
# flight sensors PX4's gz_bridge looks up by name. F450 and F450_px4 merge-include it and
# differ only in actuation.
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(quad_description)

find_package(ament_cmake REQUIRED)

# Assets only. models/F450/model.sdf sets the camera resolution and hfov that
# image_features.cpp hardcodes its focal length and pixel size against.
install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME})

ament_package()

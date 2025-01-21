cmake_minimum_required(VERSION 3.8)
project(arctos_urdf_description)

# Use C++11
add_compile_options(-std=c++11)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Include directories
include_directories(${ament_cmake_INCLUDE_DIRS})

# Install files
install(DIRECTORY 
  urdf
  meshes
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

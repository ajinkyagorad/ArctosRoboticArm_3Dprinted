cmake_minimum_required(VERSION 3.8)
project(arctos_moveit)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(arctos_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# C++ executables
add_executable(moveit_interface src/move_group_interface_coor_1.cpp)
add_executable(moveit_convert src/moveit_convert.cpp)

# Dependencies
ament_target_dependencies(moveit_interface
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pluginlib
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  geometric_shapes
  arctos_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(moveit_convert
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  pluginlib
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  geometric_shapes
  arctos_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  arctos_moveit/interface.py
  arctos_moveit/moveo_objrec_publisher.py
  arctos_moveit/transform.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ executables
install(TARGETS
  moveit_interface
  moveit_convert
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

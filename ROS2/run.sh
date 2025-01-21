#!/bin/bash

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the main demo
ros2 launch arctos_config demo.launch.py

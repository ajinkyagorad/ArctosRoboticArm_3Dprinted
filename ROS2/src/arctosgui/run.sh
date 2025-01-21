#!/bin/bash

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/Desktop/Robot/ArctosARM/ROS2/install/setup.bash

# Launch ROS2 demo in a new terminal
gnome-terminal --tab -- bash -c "source /opt/ros/humble/setup.bash; source ~/Desktop/Robot/ArctosARM/ROS2/install/setup.bash; ros2 launch arctos_config demo.launch.py; exec bash"

# Launch MoveIt interface in a new terminal
gnome-terminal --tab -- bash -c "source /opt/ros/humble/setup.bash; source ~/Desktop/Robot/ArctosARM/ROS2/install/setup.bash; ros2 run arctos_moveit interface; exec bash"

# Launch GUI in the current terminal
python3 ui.py

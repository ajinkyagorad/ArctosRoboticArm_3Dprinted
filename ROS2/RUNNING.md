# 🎮 Running Guide

## 🚀 Quick Start

### Method 1: Using the Run Script
```bash
# Make script executable (first time only)
chmod +x run.sh

# Run the robot
./run.sh
```

### Method 2: Manual Launch
```bash
# Source workspace (if not already done)
source install/setup.bash

# Launch the demo
ros2 launch arctos_config demo.launch.py
```
```markdown
### Instructions to Run the Setup

```bash
# Deactivate Conda (if active)
conda deactivate

# Ensure Python 3.10 is installed and active
python3 --version

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace
source ~/ros2_ws/install/setup.bash

# Run the MoveIt demo
ros2 launch arctos_config demo.launch.py
```

---

### Commands to Close, Kill Processes, and Restart Fresh

```bash
# List and Kill All ROS 2 Processes
pkill -9 -f ros2

# Verify All ROS 2 Processes are Terminated
ps aux | grep ros2 | grep -v grep

# Clear ROS 2 Logs and State
rm -rf ~/.ros/log

# Restart ROS 2 Daemon
ros2 daemon stop
ros2 daemon start

# Start Fresh Setup: Repeat the steps from Instructions to Run the Setup
```
# Start Fresh Setup: Repeat the steps from Instructions to Run the Setup

## 🎯 Common Operations

### 1. View Robot State
```bash
# View joint states
ros2 topic echo /joint_states

# View TF transforms
ros2 run tf2_tools view_frames
```

### 2. Control Robot
```bash
# Send a joint state command
ros2 topic pub /joint_states sensor_msgs/msg/JointState "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []"
```

### 3. Load Custom RViz Configuration
In RViz:
1. File > Open Config
2. Navigate to `src/arctos_config/config/arctosgui_config.rviz`
3. Click "Open"

Or from command line:
```bash
ros2 launch arctos_config demo.launch.py rviz_config:=src/arctos_config/config/arctosgui_config.rviz
```

## 🔧 Hardware Connection

### Arduino Connection
```bash
# Check USB connection
ls /dev/ttyUSB*  # or /dev/ttyACM*

# Set permissions (if needed)
sudo chmod a+rw /dev/ttyUSB0  # or ttyACM0

# Launch with hardware
ros2 launch arctos_config demo.launch.py use_fake_hardware:=false
```

### CANable Connection
```bash
# Check CAN interface
ip link show can0

# If not up, set up CAN interface
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set up can0

# Monitor CAN traffic (optional)
candump can0
```

## 📊 Debugging

### 1. Check Node Graph
```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# List all services
ros2 service list
```

### 2. Check Transforms
```bash
# View transform tree
ros2 run tf2_tools view_frames

# Echo transforms
ros2 topic echo /tf
```

### 3. Check Controllers
```bash
# List controllers
ros2 control list_controllers

# List hardware interfaces
ros2 control list_hardware_interfaces
```

### 4. Common Issues

#### RViz Not Showing Robot
```bash
# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch arctos_config demo.launch.py
```

#### Robot Not Moving
1. Check hardware connection:
```bash
# For Arduino
ls /dev/ttyUSB*

# For CAN
ip link show can0
```

2. Check controller status:
```bash
ros2 control list_controllers
```

## 🎓 Advanced Usage

### 1. Custom Configurations
```bash
# Launch with custom controller config
ros2 launch arctos_config demo.launch.py controllers_file:=path/to/custom_controllers.yaml

# Launch with custom URDF
ros2 launch arctos_config demo.launch.py urdf_file:=path/to/custom.urdf.xacro
```

### 2. Debug Mode
```bash
# Launch with debug output
ros2 launch arctos_config demo.launch.py log_level:=debug

# Launch with specific debug topics
ros2 launch arctos_config demo.launch.py debug_topics:="[/joint_states, /tf]"
```

### 3. Record and Playback
```bash
# Record all topics
ros2 bag record -a

# Playback recorded data
ros2 bag play path/to/rosbag2_YYYY_MM_DD-HH_MM_SS
```

## 🎉 Success Criteria

You know everything is working when:
1. RViz shows the robot model ✅
2. Joint states are being published ✅
3. MoveIt planning works ✅
4. Hardware responds to commands (if connected) ✅

## 🆘 Need Help?

1. Check the logs:
```bash
ros2 launch arctos_config demo.launch.py log_level:=debug
```

2. Check the [Issues](https://github.com/your-repo/ArctosARM/issues) page
3. Contact the maintainers

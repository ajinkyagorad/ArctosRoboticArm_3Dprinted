# 🤖 Arctos ARM ROS2

This repository contains the ROS2 implementation of the Arctos ARM robotic manipulator project. Successfully migrated from ROS1 using Windsurf, the world's first agentic IDE! 🚀

## ✨ Prerequisites

- 💻 Ubuntu 22.04 or later
- 🌟 ROS2 Humble
- 🐍 Python 3.8 or later
- 🎮 NVIDIA drivers (for visualization)

## 🛠️ Installation

1. First, install ROS2 Humble following the [official installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. Create a ROS2 workspace and clone this repository:
```bash
mkdir -p ~/arctos_ws/src
cd ~/arctos_ws/src
git clone https://github.com/your-repo/ArctosARM.git
cd ..
```

3. Install dependencies:
```bash
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control
```

4. Build the workspace:
```bash
colcon build
```

5. Source the workspace:
```bash
source install/setup.bash
```

## 🚀 Running the Robot

### 🎯 Method 1: Using the run script

1. Make the run script executable:
```bash
chmod +x run.sh
```

2. Run the script:
```bash
./run.sh
```

### 🎮 Method 2: Manual launch

1. Source your ROS2 workspace:
```bash
source install/setup.bash
```

2. Launch the main demo:
```bash
ros2 launch arctos_config demo.launch.py
```

### 🖥️ Loading RViz Configuration

1. In RViz:
   - Click `File > Open Config`
   - Navigate to `src/arctos_config/config/arctosgui_config.rviz`
   - Click "Open"

### 🔌 Connecting the Hardware

1. For Arduino Connection:
   - Connect the Arduino to your computer via USB
   - Ensure you have the correct permissions:
     ```bash
     sudo usermod -a -G dialout $USER
     sudo chmod a+rw /dev/ttyUSB0  # or ttyACM0
     ```

2. For CANable Connection:
   - Connect the CANable device
   - Set up CAN interface:
     ```bash
     sudo ip link set can0 up type can bitrate 1000000
     sudo ip link set up can0
     ```

## 📁 Project Structure

- 📂 `src/arctos_urdf_description/`: Robot URDF files and meshes
- 📂 `src/arctos_config/`: Launch files and configuration
- 📂 `src/arctos_moveit/`: MoveIt configuration and scripts
- 📂 `src/arctos_msgs/`: Custom message definitions

## 🔧 Troubleshooting

1. If RViz fails to start with OpenGL errors:
   - Ensure NVIDIA drivers are properly installed
   - Try running: `export LIBGL_ALWAYS_SOFTWARE=1`

2. If the robot doesn't move:
   - Check USB/CAN connections
   - Verify hardware is powered on
   - Check controller configuration in `config/controllers.yaml`

## 🤝 Contributing

Please read CONTRIBUTING.md for details on our code of conduct and the process for submitting pull requests.

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---
🔄 Successfully migrated from ROS1 to ROS2 using [Windsurf](https://www.codeium.com/windsurf), the world's first agentic IDE! 
Powered by Codeium's revolutionary AI Flow paradigm. 🎉

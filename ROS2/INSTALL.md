# 🛠️ Installation Guide

## 📋 Prerequisites

### 1. Install Ubuntu 22.04 LTS
Make sure you have a fresh installation of Ubuntu 22.04 LTS.

### 2. Install ROS2 Humble

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop-full -y
sudo apt install ros-dev-tools -y
```

### 3. Install Dependencies

```bash
# Install ROS2 packages
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control

# Install Python dependencies
pip3 install -U \
    transforms3d \
    numpy \
    scipy \
    PyYAML

# Install system dependencies
sudo apt install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep
```

## 🚀 Setting Up the Workspace

### 1. Create Workspace
```bash
mkdir -p ~/arctos_ws/src
cd ~/arctos_ws/src
```

### 2. Clone Repository
```bash
git clone https://github.com/your-repo/ArctosARM.git .
cd ..
```

### 3. Install Dependencies
```bash
# Initialize rosdep
sudo rosdep init
rosdep update

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Workspace
```bash
# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 5. Setup Environment
Add this to your `~/.bashrc`:
```bash
# Add to end of ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/arctos_ws/install/setup.bash" >> ~/.bashrc
```

## 🔧 Hardware Setup

### Arduino Setup
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set permissions for USB port
sudo chmod a+rw /dev/ttyUSB0  # or ttyACM0
```

### CANable Setup
```bash
# Install can-utils
sudo apt install -y can-utils

# Set up CAN interface
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set up can0
```

## ✅ Verify Installation

```bash
# Source workspace
source ~/.bashrc

# Try running the demo launch file
ros2 launch arctos_config demo.launch.py
```

If you see RViz open with the robot model, the installation was successful! 🎉

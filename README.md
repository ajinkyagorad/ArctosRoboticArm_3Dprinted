# Arctos Robotics Arm Control System 🚀🤖

## Overview 🌐
This project leverages the MKS SERVO42/57D CAN-based closed-loop stepper motor drivers for precise control of a 6-DOF robotic arm. The ultimate goal is to integrate AI and vision-based control for advanced automation. 💡📷

### Relevant Resources 🔗
- [Arctos Robotics Official Documentation](https://arctosrobotics.com/) 🌐
- [MKS SERVO42/57D User Manual](https://github.com/makerbase-motor/MKS-SERVO57D/blob/master/User%20Manual/MKS%20SERVO42%2657D_CAN%20User%20Manual%20V1.0.6.pdf) 📘
- [Stepper Motor Kit Purchase Link](https://www.aliexpress.com/item/1005005642131551.html) 🛒

---

## Hardware Setup ⚙️
1. **Components**:
   - MKS SERVO42/57D Drivers ⚡
   - CANable USB-to-CAN module 🔌
   - 6-DOF robotic arm 🦾
   - Bambulab A1 and A1 Mini 3D printers for manufacturing parts. 🖨️
2. **Connections**:
   - Motor IDs: 0x01 to 0x06 for X, Y, Z, A, B, and C joints, respectively.
   - CAN bus wiring with proper termination (120 Ω resistor). 📶
   
---

## Software Setup 🖥️
### Prerequisites 📋
- **Operating System**: Ubuntu 22.04 LTS 🐧
- **Dependencies**:
  ```bash
  sudo apt update
  sudo apt install -y python3 python3-pip can-utils colcon-common-extensions
  pip3 install python-can[serial]
  ```

### Setting Up CAN Interface 🔧
```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan
sudo slcan_attach -o -s6 -c /dev/ttyACM4
sudo slcand ttyACM4 slcan0
sudo ip link set slcan0 up
```

### Testing CAN Communication 📡
To test communication, send a dummy command:
```bash
cansend slcan0 001#FDFF400A000A0051
```
This moves the X joint of the arm. Adjust CRC for other joints. 

---

## Understanding Commands 🧠
Each CAN message consists of:
- **ID**: Motor ID (e.g., `0x01` for X).
- **Data Length**: 8 bytes.
- **Data**:
  - `FD`: Command type (relative motion).
  - `FF`: Direction and speed.
  - `400A`: Acceleration.
  - `000A`: Pulses (gear ratio considered).
  - `CRC`: Last byte (calculated dynamically).

Example for Y joint (ID=0x02):
```bash
cansend slcan0 002#FDFF400A000A0052
```

---

## Current Progress ✅
1. **Initialization**: Commands successfully enable motors and verify communication. 🟢
2. **Motion Testing**: `cansend` used to test movements with calculated CRCs. ✅
3. **Angle Calculation**: Commands generated for precise joint rotation based on gear ratios. ⚙️

---

## Future Plans 🛠️
- Integrate vision-based AI for real-time control. 📷🤖
- Develop inverse kinematics for end-effector positioning. 🧮
- Extend system for dynamic path planning and obstacle avoidance. 🚧

---

## Demonstration 📽️
[![Watch the video](https://img.youtube.com/vi/yEWwH9ZDHyo/hqdefault.jpg)](https://www.youtube.com/shorts/yEWwH9ZDHyo)

---

## Acknowledgments 🙌
Special thanks to:
- [Arctos Robotics](https://github.com/Arctos-Robotics) for the foundational framework.
- Makerbase for the robust MKS servo drivers. 🛠️

---

## Contributing 🤝
We welcome contributions! Feel free to open issues or submit pull requests. 💌


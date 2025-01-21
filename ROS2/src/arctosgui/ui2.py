import os
import tkinter as tk
from tkinter import ttk
import serial.tools.list_ports
import can
from ttkthemes import ThemedStyle
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from PIL import Image, ImageTk
import sv_ttk
import math

# Change this according to your folder
user_path = "/home/triton/Desktop/Robot/ArctosARM/arctosgui"

# Global variables
selected_port = None
connected = False
bus = None
last_joint_states = None
last_cartesian_position = None

class ArctosControlGUI(Node):
    def __init__(self):
        super().__init__('arctos_control_gui')

        # ROS Publishers
        self.ui_command_pub = self.create_publisher(String, '/ui_command', 10)

        # ROS Subscribers
        self.create_subscription(JointState, '/joint_states', self.update_joint_sliders, 10)
        self.create_subscription(TransformStamped, '/transformed_tf', self.update_cartesian_sliders, 10)

    def update_joint_sliders(self, data):
        """ Update joint sliders based on received joint state data """
        global last_joint_states
        if last_joint_states is None or data.position != last_joint_states.position:
            last_joint_states = data
            for i, joint in enumerate(data.name):
                if joint in joint_name_to_slider:
                    slider = joint_name_to_slider[joint]
                    slider.set(data.position[i])

    def update_cartesian_sliders(self, data):
        """ Update Cartesian sliders based on received transformed tf data """
        global last_cartesian_position
        if last_cartesian_position is None or \
           data.transform.translation.x != last_cartesian_position.transform.translation.x or \
           data.transform.translation.y != last_cartesian_position.transform.translation.y or \
           data.transform.translation.z != last_cartesian_position.transform.translation.z:
            last_cartesian_position = data
            cartesian_path_sliders[0].set(data.transform.translation.x)
            cartesian_path_sliders[1].set(data.transform.translation.y)
            cartesian_path_sliders[2].set(data.transform.translation.z)

    def publish_ui_command(self, command):
        msg = String()
        msg.data = command
        self.ui_command_pub.publish(msg)
        print(f"Published: {command}")

# Tkinter GUI Setup
root = tk.Tk()
root.title("Arctos CAN Controller")

# Define helper functions for interacting with ROS
ros_node = None  # Global ROS Node

def go_to_joint_state():
    joint_state_values = [slider.get() for slider in joint_state_sliders]
    ros_node.publish_ui_command("go_to_joint_state," + ','.join(map(str, joint_state_values)))

def plan_cartesian_path():
    cartesian_path_values = [slider.get() for slider in cartesian_path_sliders]
    ros_node.publish_ui_command("plan_cartesian_path," + ','.join(map(str, cartesian_path_values)))

def open_gripper():
    ros_node.publish_ui_command("open_gripper")

def close_gripper():
    ros_node.publish_ui_command("close_gripper")

# GUI Component Definitions
# Refresh Ports, Connect, Disconnect, etc.
def refresh_ports():
    ports = [port.device for port in serial.tools.list_ports.comports()]
    port_combobox['values'] = ports
    if ports:
        port_combobox.current(0)

def connect():
    global selected_port, connected, bus
    port = port_combobox.get()

    if not port:
        update_message("Please select a port.")
        return

    try:
        bus = can.interface.Bus(bustype="slcan", channel=port, bitrate=50000)
        connected = True
        selected_port = port
        update_message(f"Connected to port {port}.")
    except Exception as e:
        update_message(f"Error connecting: {str(e)}")

def disconnect():
    global connected, bus
    if connected:
        try:
            bus.shutdown()
            connected = False
            update_message(f"Disconnected from port {selected_port}.")
        except Exception as e:
            update_message(f"Error disconnecting: {str(e)}")
    else:
        update_message("Not connected to any port.")

def update_message(message):
    messages_text.config(state=tk.NORMAL)
    messages_text.insert(tk.END, message + "\n")
    messages_text.see(tk.END)
    messages_text.config(state=tk.DISABLED)
    print("Message updated:", message)

# Joint State Pose Sliders
joint_state_label = ttk.Label(root, text="Joint jog:")
joint_state_label.grid(row=8, column=0, columnspan=2, padx=5, pady=5, sticky="w")
joint_state_sliders = []
joint_name_to_slider = {}
for i, label in enumerate(['X', 'Y', 'Z', 'A', 'B', 'C']):
    ttk.Label(root, text=label).grid(row=i+9, column=0, padx=5, pady=5, sticky='w')
    slider = ttk.Scale(root, from_=-3.14, to=3.14, orient=tk.HORIZONTAL, length=200)
    slider.set(0)
    slider.grid(row=i+9, column=1, padx=5, pady=5)
    joint_state_sliders.append(slider)
    joint_name_to_slider[f"joint{i+1}"] = slider

# Cartesian Path Sliders
cartesian_path_label = ttk.Label(root, text="Tool jog:")
cartesian_path_label.grid(row=15, column=0, columnspan=2, padx=5, pady=5, sticky="w")
cartesian_path_sliders = []
for i, label in enumerate(['X', 'Y', 'Z']):
    ttk.Label(root, text=label).grid(row=i+16, column=0, padx=5, pady=5, sticky='w')
    slider = ttk.Scale(root, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, length=200)
    slider.set(0)
    slider.grid(row=i+16, column=1, padx=5, pady=5)
    cartesian_path_sliders.append(slider)

# Buttons for Actions
joint_state_button = ttk.Button(root, text="Move joints", command=go_to_joint_state)
joint_state_button.grid(row=19, column=0, padx=5, pady=(5, 10))

cartesian_path_button = ttk.Button(root, text="Move tool", command=plan_cartesian_path)
cartesian_path_button.grid(row=19, column=1, padx=5, pady=(5, 10))

open_gripper_button = ttk.Button(root, text="Open Gripper", command=open_gripper)
open_gripper_button.grid(row=19, column=3, padx=5, pady=(5, 10), sticky="e")

close_gripper_button = ttk.Button(root, text="Close Gripper", command=close_gripper)
close_gripper_button.grid(row=19, column=3, padx=(5, 10), pady=(5, 10), sticky="w")

# Messages Display
messages_text = tk.Text(root, height=8, width=50, state=tk.DISABLED)
messages_text.grid(row=20, column=0, columnspan=5, padx=5, pady=(0, 5), sticky="ew")

# Set theme light/dark
sv_ttk.set_theme("light")

# Main Function to Start ROS Node and GUI
def main():
    global ros_node

    # Initialize ROS 2
    rclpy.init()
    ros_node = ArctosControlGUI()

    # Run the Tkinter GUI in a separate thread to allow ROS 2 spinning
    def ros_spin():
        rclpy.spin(ros_node)

    import threading
    threading.Thread(target=ros_spin, daemon=True).start()

    # Start the Tkinter GUI
    root.mainloop()

    # Cleanup
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

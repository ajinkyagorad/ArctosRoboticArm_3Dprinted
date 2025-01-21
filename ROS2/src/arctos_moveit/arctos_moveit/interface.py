#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from std_msgs.msg import String
from arctos_msgs.msg import JointAngles, RobotPose
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
import copy
import math

class MoveGroupPythonInterfaceTutorial(Node):
    """MoveGroupPythonInterfaceTutorial"""
    def __init__(self):
        super().__init__('ros_node')
        
        # ROS initialization
        self.create_subscription(String, "ui_command", self.ui_command_callback, 10)
        
        # MoveIt initialization
        self.robot = None
        self.scene = None
        self.group = None
        self.gripper_group = None
        
        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Move Group Python Interface Tutorial Node Started')

    def ui_command_callback(self, data):
        command = data.data.split(',')
        if command[0] == "go_to_joint_state":
            joint_state_values = [float(value) for value in command[1:]]
            self.go_to_joint_state(joint_state_values)

        elif command[0] == "plan_cartesian_path":
            cartesian_path_values = [float(value) for value in command[1:]]
            self.plan_cartesian_path(cartesian_path_values)

        elif command[0] == "open_gripper":
            self.open_gripper()

        elif command[0] == "close_gripper":
            self.close_gripper()

        else:
        # Unknown command
            self.get_logger().warn("Unknown command received: %s", command)

    def go_to_joint_state(self, joint_state_values):
        self.group.go(joint_state_values, wait=True)
        self.group.stop()

    def plan_cartesian_path(self, cartesian_path_values):
        waypoints = []
        for i in range(0, len(cartesian_path_values), 3):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = cartesian_path_values[i:i+3]
            waypoints.append(copy.deepcopy(pose))

        (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.group.execute(plan, wait=True)

    def open_gripper(self):
        gripper_values = [0.0, 0.0]  # Predefined values for opening the gripper
        self.gripper_group.go(gripper_values, wait=True)
        self.gripper_group.stop()
        self.get_logger().info("Gripper opened")

    def close_gripper(self):
        gripper_values = [0.015, 0.015]  # Predefined values for closing the gripper
        self.gripper_group.go(gripper_values, wait=True)
        self.gripper_group.stop()
        self.get_logger().info("Gripper closed")

def main(args=None):
    rclpy.init(args=args)
    node = MoveGroupPythonInterfaceTutorial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

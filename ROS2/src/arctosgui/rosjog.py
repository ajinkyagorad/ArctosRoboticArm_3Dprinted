import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointListener(Node):
    def __init__(self):
        super().__init__('joint_listener')
        self.previous_joint_values = None
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # Prevent unused variable warning.

    def joint_states_callback(self, msg):
        try:
            # Similar joint conversion logic as in the original script
            joint1_degrees = math.degrees(msg.position[msg.name.index('joint1')])
            joint2_degrees = math.degrees(msg.position[msg.name.index('joint2')])
            joint3_degrees = math.degrees(msg.position[msg.name.index('joint3')])
            joint4_degrees = math.degrees(msg.position[msg.name.index('joint4')])
            joint5_degrees = math.degrees(msg.position[msg.name.index('joint5')])
            joint6_degrees = math.degrees(msg.position[msg.name.index('joint6')])

            b_axis = (joint6_degrees - joint5_degrees) / 2
            c_axis = (joint5_degrees + joint6_degrees) / 2

            joint_values = (
                round(joint1_degrees, 2),
                round(joint2_degrees, 2),
                round(joint3_degrees, 2),
                round(joint4_degrees, 2),
                round(b_axis, 2),
                round(c_axis, 2),
            )

            if joint_values != self.previous_joint_values:
                with open('jog.tap', 'w') as file:
                    file.write("F800\n")
                    file.write(f"G90 X {joint_values[0]} Y {joint_values[1]} Z {joint_values[2]} A {joint_values[3]} B {joint_values[4]} C {joint_values[5]}\n")
                self.previous_joint_values = joint_values

        except ValueError:
            self.get_logger().warn("Could not find all required joint names in message.")

def main():
    rclpy.init()
    node = JointListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

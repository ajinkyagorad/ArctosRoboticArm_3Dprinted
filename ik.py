import numpy as np
import can

class RoboticArm6DOF:
    def __init__(self, dh_params, joint_limits, can_channel='can0', can_bitrate=500000):
        """
        Initialize the robotic arm.

        Parameters:
        - dh_params: List of Denavit-Hartenberg parameters for each joint (a, alpha, d, theta).
        - joint_limits: Tuple of (min, max) for each joint.
        - can_channel: The CAN channel (default is 'can0').
        - can_bitrate: The bitrate for CAN communication (default is 500000).
        """
        self.dh_params = dh_params
        self.joint_limits = joint_limits
        self.can_bus = can.interface.Bus(bustype='socketcan', channel=can_channel, bitrate=can_bitrate)

    def dh_transform(self, a, alpha, d, theta):
        """
        Compute the transformation matrix using DH parameters.
        """
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),              d],
            [0,              0,                          0,                          1]
        ])

    def forward_kinematics(self, joint_angles):
        """
        Compute the end-effector position and orientation using forward kinematics.

        Parameters:
        - joint_angles: List of joint angles in radians.

        Returns:
        - 4x4 Transformation matrix of the end-effector.
        """
        T = np.eye(4)
        for i, (a, alpha, d, theta) in enumerate(self.dh_params):
            theta += joint_angles[i]  # Add joint angle to theta
            T = np.dot(T, self.dh_transform(a, alpha, d, theta))
        return T

    def calculate_trajectory(self, current_angles, target_angles, max_velocity, time_step):
        """
        Calculate joint velocities to follow a trajectory.

        Parameters:
        - current_angles: List of current joint angles in radians.
        - target_angles: List of target joint angles in radians.
        - max_velocity: Maximum joint velocity in radians/sec.
        - time_step: Time step for velocity calculation.

        Returns:
        - List of joint velocities for each joint.
        """
        joint_velocities = []
        for current, target in zip(current_angles, target_angles):
            delta = target - current
            velocity = np.clip(delta / time_step, -max_velocity, max_velocity)
            joint_velocities.append(velocity)
        return joint_velocities

    def send_can_message(self, joint_id, position, velocity):
        """
        Send a CAN message to control a joint.

        Parameters:
        - joint_id: ID of the joint to control.
        - position: Target position in encoder ticks or radians.
        - velocity: Target velocity in ticks/s or rad/s.
        """
        # Example CAN data encoding (modify based on protocol)
        position_data = int(position * 1000).to_bytes(4, byteorder='little', signed=True)
        velocity_data = int(velocity * 1000).to_bytes(4, byteorder='little', signed=True)
        data = [joint_id] + list(position_data) + list(velocity_data)
        message = can.Message(arbitration_id=0x100 + joint_id, data=data, is_extended_id=False)
        self.can_bus.send(message)

    def control_arm(self, current_angles, target_angles, max_velocity, time_step):
        """
        Calculate velocities and send CAN messages to control the arm.

        Parameters:
        - current_angles: List of current joint angles in radians.
        - target_angles: List of target joint angles in radians.
        - max_velocity: Maximum joint velocity in radians/sec.
        - time_step: Time step for velocity calculation.
        """
        velocities = self.calculate_trajectory(current_angles, target_angles, max_velocity, time_step)
        for joint_id, (position, velocity) in enumerate(zip(target_angles, velocities), start=1):
            self.send_can_message(joint_id, position, velocity)

# Example usage:
if __name__ == "__main__":
    # Define DH parameters: [(a, alpha, d, theta)] for each joint
    dh_params = [
        (0, 0, 287.87 / 1000, 0),  # Joint 1 (convert mm to meters)
        (20.174 / 1000, -np.pi/2, 0, -np.pi/2),  # Joint 2
        (260.986 / 1000, 0, 0, 0),  # Joint 3
        (19.219 / 1000, 0, 260.753 / 1000, 0),  # Joint 4
        (0, np.pi/2, 0, np.pi/2),  # Joint 5
        (0, -np.pi/2, 74.745 / 1000, np.pi)  # Joint 6
    ]

    # Joint limits (in radians)
    joint_limits = [(-np.pi, np.pi) for _ in range(6)]

    # Initialize the arm
    arm = RoboticArm6DOF(dh_params, joint_limits)

    # Current and target joint states
    current_angles = [0, 0, 0, 0, 0, 0]
    target_angles = [np.pi/4, -np.pi/6, np.pi/6, -np.pi/4, np.pi/8, -np.pi/8]

    # Control the arm
    arm.control_arm(current_angles, target_angles, max_velocity=1.0, time_step=0.1)

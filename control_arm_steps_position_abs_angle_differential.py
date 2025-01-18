import time
import can
import tkinter as tk
from tkinter import ttk
from helper_motor import (
    start_motor_position_mode4,  # Absolute position mode
    stop_motor_position_mode4,  # Stop motor in position mode
    read_encoder_carry_value,  # Get encoder value
    enable_motor,  # Enable motor
    set_current_axis_to_zero,  # Set current axis to zero
    set_working_current  # Set working current
)

# Initialize the CAN bus
bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)

# Define motor IDs
motor_ids = [1, 2, 3, 4, 5, 6]  # IDs of the six stepers
gear_ratios = [6.75, 75, 75, 24, 33.91, 33.91]  # Gear ratios for motors
k_factor = [166.67, 166.67, 166.67, 166.67, 63.27, 63.27]  # pulses (/200 ) per degree
steps_per_revolution = 200  # Steps per revolution for stepper motors

# Calculate pulses per degree
pulses_per_degree = [
    (gear_ratios[i] * k_factor[i] * steps_per_revolution) / 360 for i in range(len(motor_ids))
]

def angle_to_pulse(motor_id, angle):
    """Convert angle to pulses based on gear ratio."""
    return int(angle * pulses_per_degree[motor_id - 1])

def pulse_to_angle(motor_id, pulses):
    """Convert pulses to angle based on gear ratio."""
    return pulses / pulses_per_degree[motor_id - 1]

def move_motor_to_position(motor_id, target_angle):
    """Move the motor to an absolute position in degrees."""
    try:
        speed = 1000  # Default speed
        acceleration = 10  # Default acceleration
        target_pulses = angle_to_pulse(motor_id, target_angle)
        start_motor_position_mode4(bus, motor_id, speed=speed, acc=acceleration, abs_axis=target_pulses)
        print(f"Motor {motor_id} moving to {target_angle}° ({target_pulses} pulses)")
    except Exception as e:
        print(f"Error moving motor {motor_id}: {e}")

def move_m5_m6(theta_from_side, theta_from_front):
    """Move motors 5 and 6 using the differential drive equations."""
    try:
        # Calculate angles for motors 5 and 6
        theta_m5 = theta_from_side +  theta_from_front
        theta_m6 = -theta_from_side + theta_from_front

        # Move motors 5 and 6
        move_motor_to_position(5, theta_m5)
        move_motor_to_position(6, theta_m6)
        print(f"Motor 5 moving to {theta_m5}° | Motor 6 moving to {theta_m6}°")
    except Exception as e:
        print(f"Error moving motors 5 and 6: {e}")

def stop_all_motors():
    """Stop all motors."""
    for motor_id in motor_ids:
        try:
            stop_motor_position_mode4(bus, motor_id, acc=0)
        except Exception as e:
            print(f"Error stopping motor {motor_id}: {e}")

def set_all_axes_to_zero():
    """Set the encoder values of all motors to zero."""
    for motor_id in motor_ids:
        try:
            set_current_axis_to_zero(bus, motor_id)
            print(f"Motor {motor_id} axis set to zero.")
        except Exception as e:
            print(f"Error setting motor {motor_id} axis to zero: {e}")

def create_motor_control_frame(root, motor_id):
    """Create a control frame for each motor."""
    frame = ttk.LabelFrame(root, text=f"Motor {motor_id} Control")
    frame.pack(fill="x", padx=10, pady=5)

    angle_label = ttk.Label(frame, text="Target Angle (°):")
    angle_label.pack(side="left", padx=5)

    angle_entry = ttk.Entry(frame, width=10)
    angle_entry.insert(0, "0")
    angle_entry.pack(side="left", padx=5)

    def move_to_position():
        target_angle = float(angle_entry.get())
        move_motor_to_position(motor_id, target_angle)

    move_button = ttk.Button(frame, text="Move to Position", command=move_to_position)
    move_button.pack(side="left", padx=5)

    encoder_label = ttk.Label(frame, text="Encoder Value (°): --")
    encoder_label.pack(side="left", padx=5)

    def update_encoder():
        try:
            encoder_data = read_encoder_carry_value(bus, motor_id)
            if encoder_data:
                carry, value = encoder_data
                pulses = (carry << 14) + value  # Combine carry and value
                angle = pulse_to_angle(motor_id, pulses)
                encoder_label.config(text=f"Encoder: {angle:.2f}°")
        except Exception as e:
            encoder_label.config(text=f"Error: {e}")
        root.after(500, update_encoder)  # Update every 500ms

    update_encoder()  # Start updating the encoder value

def create_m5_m6_frame(root):
    """Create a control frame for differential drive motors (M5 and M6)."""
    frame = ttk.LabelFrame(root, text="Differential Drive Control (M5 & M6)")
    frame.pack(fill="x", padx=10, pady=5)

    side_label = ttk.Label(frame, text="Theta from Side (°):")
    side_label.pack(side="left", padx=5)

    side_entry = ttk.Entry(frame, width=10)
    side_entry.insert(0, "0")
    side_entry.pack(side="left", padx=5)

    front_label = ttk.Label(frame, text="Theta from Front (°):")
    front_label.pack(side="left", padx=5)

    front_entry = ttk.Entry(frame, width=10)
    front_entry.insert(0, "0")
    front_entry.pack(side="left", padx=5)

    def move_m5_m6_positions():
        theta_from_side = float(side_entry.get())
        theta_from_front = float(front_entry.get())
        move_m5_m6(theta_from_side, theta_from_front)

    move_button = ttk.Button(frame, text="Move M5 & M6", command=move_m5_m6_positions)
    move_button.pack(side="left", padx=5)

def main():
    """Main function to create the GUI."""
    root = tk.Tk()
    root.title("6DOF Robot Arm Absolute Position Control")

    # Create motor control frames
    for motor_id in motor_ids[:4]:  # Motors 1 to 4
        create_motor_control_frame(root, motor_id)
        enable_motor(bus, motor_id, enable=True)
        set_working_current(bus, motor_id, current_ma=3000)

    # Create differential control frame for M5 & M6
    create_m5_m6_frame(root)
    enable_motor(bus, 5, enable=True)
    enable_motor(bus, 6, enable=True)
    set_working_current(bus, 5, current_ma=3000)
    set_working_current(bus, 6, current_ma=3000)

    # Add global stop button
    stop_button = ttk.Button(root, text="STOP ALL", command=stop_all_motors)
    stop_button.pack(fill="x", padx=10, pady=10)

    # Add set axes to zero button
    zero_button = ttk.Button(root, text="SET ALL AXES TO ZERO", command=set_all_axes_to_zero)
    zero_button.pack(fill="x", padx=10, pady=10)

    # Start the GUI loop
    root.mainloop()

    # Ensure the CAN bus is properly shut down
    bus.shutdown()

if __name__ == "__main__":
    # Enable all motors
    for motor_id in motor_ids:
        enable_motor(bus, motor_id, enable=True)
    print("All motors enabled.")
    main()

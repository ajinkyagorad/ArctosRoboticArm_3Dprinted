import time
import can
import tkinter as tk
from tkinter import ttk
from helper_motor import (
    start_motor_position_mode,  # Relative position mode
    start_motor_position_mode4,  # Absolute position mode
    stop_motor_position_mode4,  # Stop motor in absolute mode
    stop_motor_position_mode,  # Stop motor in relative mode
    read_encoder_carry_value,  # Get encoder value
    enable_motor,  # Enable motor
    set_current_axis_to_zero,  # Set encoder to zero
    set_working_current  # Set working current
)

# Initialize the CAN bus
bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)

# Motor and gear configuration
motor_ids = [1, 2, 3, 4, 5, 6]
gear_ratios = [6.75, 75, 75, 24, 33.91, 33.91]
k_factor = [166.67, 166.67, 166.67, 166.67, 63.27, 63.27]
steps_per_revolution = 200

# Pulses per degree calculation
pulses_per_degree = [
    (gear_ratios[i] * k_factor[i] * steps_per_revolution) / 360 for i in range(len(motor_ids))
]

def angle_to_pulse(motor_id, angle):
    """Convert angle to pulses."""
    return int(angle * pulses_per_degree[motor_id - 1])

def pulse_to_angle(motor_id, pulses):
    """Convert pulses to angle."""
    return pulses / pulses_per_degree[motor_id - 1]

# Motor control functions
def move_motor_absolute(motor_id, target_angle):
    """Move motor to an absolute position."""
    try:
        speed = 1000
        acceleration = 10
        pulses = angle_to_pulse(motor_id, target_angle)
        start_motor_position_mode4(bus, motor_id, speed=speed, acc=acceleration, abs_axis=pulses)
        print(f"Motor {motor_id} moving to {target_angle}° ({pulses} pulses)")
    except Exception as e:
        print(f"Error moving motor {motor_id} to absolute position: {e}")

def move_motor_relative(motor_id, step_size, direction):
    """Move motor by a relative step size."""
    try:
        speed = 500
        acceleration = 10
        start_motor_position_mode(bus, motor_id, direction=direction, speed=speed, acc=acceleration, pulses=step_size)
        print(f"Motor {motor_id} moved relative by {step_size} pulses in direction {direction}")
    except Exception as e:
        print(f"Error moving motor {motor_id} relative: {e}")

def stop_all_motors():
    """Stop all motors."""
    for motor_id in motor_ids:
        try:
            stop_motor_position_mode4(bus, motor_id, acc=0)
        except Exception as e:
            print(f"Error stopping motor {motor_id}: {e}")

def clear_all_encoders():
    """Set all encoders to zero."""
    for motor_id in motor_ids:
        try:
            set_current_axis_to_zero(bus, motor_id)
            print(f"Encoder for motor {motor_id} cleared to zero.")
        except Exception as e:
            print(f"Error clearing encoder for motor {motor_id}: {e}")

# Differential control for M5 and M6
def move_m5_m6(theta_from_side, theta_from_front):
    """Differential control for M5 and M6."""
    theta_m5 = theta_from_side + theta_from_front
    theta_m6 = -theta_from_side + theta_from_front
    move_motor_absolute(5, theta_m5)
    move_motor_absolute(6, theta_m6)
    print(f"Motor 5: {theta_m5}°, Motor 6: {theta_m6}°")

def create_motor_control_frame(root, motor_id, angle_entries, step_entries):
    """Create a control frame for each motor."""
    frame = ttk.LabelFrame(root, text=f"Motor {motor_id} Control")
    frame.pack(fill="x", padx=10, pady=5)

    # Absolute position controls
    abs_label = ttk.Label(frame, text="Absolute Position (°):")
    abs_label.pack(side="left", padx=5)

    abs_entry = ttk.Entry(frame, width=10)
    abs_entry.insert(0, "0")
    abs_entry.pack(side="left", padx=5)
    angle_entries[motor_id] = abs_entry

    abs_button = ttk.Button(frame, text="Set Abs Pos", command=lambda: move_motor_absolute(motor_id, float(abs_entry.get())))
    abs_button.pack(side="left", padx=5)

    # Relative position controls
    rel_label = ttk.Label(frame, text="Step Size (pulses):")
    rel_label.pack(side="left", padx=5)

    rel_entry = ttk.Entry(frame, width=10)
    rel_entry.insert(0, "0")
    rel_entry.pack(side="left", padx=5)
    step_entries[motor_id] = rel_entry

    rel_forward_button = ttk.Button(frame, text="Forward", command=lambda: move_motor_relative(motor_id, int(rel_entry.get()), 0))
    rel_forward_button.pack(side="left", padx=5)

    rel_backward_button = ttk.Button(frame, text="Backward", command=lambda: move_motor_relative(motor_id, int(rel_entry.get()), 1))
    rel_backward_button.pack(side="left", padx=5)

    # Encoder display
    encoder_label = ttk.Label(frame, text="Encoder: --")
    encoder_label.pack(side="left", padx=5)

    def update_encoder():
        try:
            encoder_data = read_encoder_carry_value(bus, motor_id)
            if encoder_data:
                carry, value = encoder_data
                pos = (carry << 14) | value  # Combine carry and value
                angle = pulse_to_angle(motor_id, pos)
                encoder_label.config(text=f"Encoder: {angle:.2f}°")
        except Exception as e:
            encoder_label.config(text=f"Error: {e}")
        root.after(100, update_encoder)  # Update every 100ms

    update_encoder()

def create_m5_m6_frame(root):
    """Create controls for M5 and M6 differential control."""
    frame = ttk.LabelFrame(root, text="M5 & M6 Differential Control")
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

    move_button = ttk.Button(frame, text="Move M5 & M6", command=lambda: move_m5_m6(float(side_entry.get()), float(front_entry.get())))
    move_button.pack(side="left", padx=5)

def create_apply_all_button(root, angle_entries):
    """Create button to apply all motor absolute positions."""
    def apply_all_positions():
        for motor_id, entry in angle_entries.items():
            move_motor_absolute(motor_id, float(entry.get()))
    apply_button = ttk.Button(root, text="APPLY ALL", command=apply_all_positions)
    apply_button.pack(fill="x", padx=10, pady=10)

def main():
    """Main function to create the GUI."""
    root = tk.Tk()
    root.title("6DOF Robot Arm Control")
    angle_entries = {}
    step_entries = {}

    # Create control frames for each motor
    for motor_id in motor_ids:
        create_motor_control_frame(root, motor_id, angle_entries, step_entries)

    # Create differential control for M5 and M6
    create_m5_m6_frame(root)

    # Global control buttons
    ttk.Button(root, text="STOP ALL", command=stop_all_motors).pack(fill="x", padx=10, pady=10)
    ttk.Button(root, text="CLEAR ENCODERS", command=clear_all_encoders).pack(fill="x", padx=10, pady=10)
    create_apply_all_button(root, angle_entries)

    root.mainloop()
    bus.shutdown()

if __name__ == "__main__":
    # Enable all motors
    for motor_id in motor_ids:
        enable_motor(bus, motor_id, enable=True)
        set_working_current(bus, motor_id, current_ma=3000)
    print("All motors enabled.")
    main()

import time
import can
import tkinter as tk
from tkinter import ttk
from helper_motor import (
    start_motor_position_mode4,  # Absolute position mode
    stop_motor_position_mode4,  # Stop motor in position mode
    read_encoder_carry_value,  # Get encoder value
    enable_motor,  # Enable motor
    set_home_parameters,  # Set home position
    set_working_current  # Set working current
)

# Initialize the CAN bus
bus = can.interface.Bus(interface="slcan", channel="COM4", bitrate=500000)

# Define motor IDs
motor_ids = [1, 2, 3, 4, 5, 6]  # IDs of the six servos

def move_motor_to_position(motor_id, abs_position):
    """Move the motor to an absolute position."""
    try:
        speed = 1000  # Default speed
        acceleration = 10  # Default acceleration
        start_motor_position_mode4(bus, motor_id, speed=speed, acc=acceleration, abs_axis=abs_position)
    except Exception as e:
        print(f"Error moving motor {motor_id}: {e}")

def stop_all_motors():
    """Stop all motors."""
    for motor_id in motor_ids:
        try:
            stop_motor_position_mode4(bus, motor_id, acc=0)
        except Exception as e:
            print(f"Error stopping motor {motor_id}: {e}")

def set_home():
    """Set the current position of all motors as home."""
    for motor_id in motor_ids:
        try:
            set_home_parameters(bus, motor_id, home_trig=1, home_dir=0, home_speed=0, end_limit=0)
            print(f"Motor {motor_id} home set.")
        except Exception as e:
            print(f"Error setting home for motor {motor_id}: {e}")

def create_motor_control_frame(root, motor_id):
    """Create a control frame for each motor."""
    frame = ttk.LabelFrame(root, text=f"Motor {motor_id} Control")
    frame.pack(fill="x", padx=10, pady=5)

    position_label = ttk.Label(frame, text="Target Position:")
    position_label.pack(side="left", padx=5)

    position_entry = ttk.Entry(frame, width=10)
    position_entry.insert(0, "0")
    position_entry.pack(side="left", padx=5)

    def move_to_position():
        abs_position = int(position_entry.get())
        move_motor_to_position(motor_id, abs_position)

    move_button = ttk.Button(frame, text="Move to Position", command=move_to_position)
    move_button.pack(side="left", padx=5)

    encoder_label = ttk.Label(frame, text="Encoder Value: --")
    encoder_label.pack(side="left", padx=5)

    def update_encoder():
        try:
            encoder_data = read_encoder_carry_value(bus, motor_id)
            if encoder_data:
                carry, value = encoder_data
                encoder_label.config(text=f"Encoder: {carry + value}")
        except Exception as e:
            encoder_label.config(text=f"Error: {e}")
        root.after(500, update_encoder)  # Update every 500ms

    update_encoder()  # Start updating the encoder value

def main():
    """Main function to create the GUI."""
    root = tk.Tk()
    root.title("6DOF Robot Arm Absolute Position Control")

    # Create motor control frames
    for motor_id in motor_ids:
        create_motor_control_frame(root, motor_id)
        enable_motor(bus, motor_id, enable=True)
        set_working_current(bus, motor_id, current_ma=3000)

    # Add global stop button
    stop_button = ttk.Button(root, text="STOP ALL", command=stop_all_motors)
    stop_button.pack(fill="x", padx=10, pady=10)

    # Add set home button
    home_button = ttk.Button(root, text="SET CURRENT POSITION AS HOME", command=set_home)
    home_button.pack(fill="x", padx=10, pady=10)

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

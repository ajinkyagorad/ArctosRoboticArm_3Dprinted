import can
import time


def set_calibrate_encoder(bus, motor_id):
    """
    Calibrate the encoder (Command: 0x80).
    """
    data = [0x80, 0x00]  # Command and data for calibration
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Calibrating..." if status == 0 else "Calibration Success" if status == 1 else "Calibration Failed"
    return "No Response"


def set_work_mode(bus, motor_id, mode):
    """
    Set the work mode (Command: 0x82).
    Modes: 
      0: CR_OPEN, 1: CR_CLOSE, 2: CR_vFOC, 3: SR_OPEN, 4: SR_CLOSE, 5: SR_vFOC
    """
    if mode not in range(6):
        return "Invalid Mode"
    data = [0x82, mode]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_working_current(bus, motor_id, current_ma):
    """
    Set the working current (Command: 0x83).
    Maximum current: 3000mA (SERVO042D/28D/35D), 5200mA (SERVO057D)
    """
    if current_ma > 5200:
        return "Invalid Current Value"
    data = [0x83, (current_ma & 0xFF00) >> 8, current_ma & 0xFF]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_holding_current(bus, motor_id, percentage):
    """
    Set the holding current percentage (Command: 0x9B).
    Valid percentages: 0 (10%) to 8 (90%)
    """
    if percentage not in range(9):
        return "Invalid Percentage"
    data = [0x9B, percentage]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_subdivision(bus, motor_id, subdivision):
    """
    Set the subdivision (Command: 0x84).
    Valid range: 0x00 to 0xFF
    """
    if subdivision < 0x00 or subdivision > 0xFF:
        return "Invalid Subdivision Value"
    data = [0x84, subdivision]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_en_pin_active(bus, motor_id, mode):
    """
    Set the active mode of the En pin (Command: 0x85).
    Modes:
      0: Active Low, 1: Active High, 2: Active Always (Hold)
    """
    if mode not in [0, 1, 2]:
        return "Invalid Mode"
    data = [0x85, mode]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_motor_direction(bus, motor_id, direction):
    """
    Set the motor rotation direction (Command: 0x86).
    Directions:
      0: CW (Clockwise), 1: CCW (Counter-Clockwise)
    """
    if direction not in [0, 1]:
        return "Invalid Direction"
    data = [0x86, direction]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_auto_screen_off(bus, motor_id, enable):
    """
    Set auto screen off function (Command: 0x87).
    Enable:
      0: Disabled, 1: Enabled
    """
    if enable not in [0, 1]:
        return "Invalid Option"
    data = [0x87, enable]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_locked_rotor_protection(bus, motor_id, enable):
    """
    Set the motor shaft locked-rotor protection (Command: 0x88).
    Enable:
      0: Disabled, 1: Enabled
    """
    if enable not in [0, 1]:
        return "Invalid Option"
    data = [0x88, enable]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_subdivision_interpolation(bus, motor_id, enable):
    """
    Set the subdivision interpolation function (Command: 0x89).
    Enable:
      0: Disabled, 1: Enabled
    """
    if enable not in [0, 1]:
        return "Invalid Option"
    data = [0x89, enable]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def set_can_bitrate(bus, motor_id, bitrate):
    """
    Set the CAN bitrate (Command: 0x8A).
    Bitrates:
      0: 125K, 1: 250K, 2: 500K, 3: 1M
    """
    if bitrate not in [0, 1, 2, 3]:
        return "Invalid Bitrate"
    data = [0x8A, bitrate]
    response = send_can_message(bus, motor_id, data)
    if response:
        status = response[1]
        return "Set Success" if status == 1 else "Set Failed"
    return "No Response"


def calculate_crc(can_id, data):
    """Calculate the 8-bit CRC for the given CAN ID and data."""
    crc_data = [can_id & 0xFF, (can_id >> 8) & 0xFF]  # Include CAN ID (as two bytes)
    crc_data.extend(data)  # Append the message data
    return sum(crc_data) & 0xFF  # Compute the CRC


def send_can_message(bus, can_id, data, dlc=None):
    """Send a CAN message and wait for a response."""
    crc = calculate_crc(can_id, data)
    data.append(crc)
    msg = can.Message(arbitration_id=can_id, data=data, dlc=dlc, is_extended_id=False)
    
    try:
        bus.send(msg)
        print(f"Message sent: {msg}")
    except can.CanError:
        print("Message NOT sent")
        return None

    print("Waiting for response...")
    response = bus.recv(timeout=2)
    if response:
        print(f"Reply received: {response}")
        return response.data
    print("No reply received.")
    return None


def set_can_id(bus, old_can_id, new_can_id):
    """
    Set the CAN ID of the motor.

    Args:
        bus: CAN bus instance.
        old_can_id: Current CAN ID of the motor.
        new_can_id: New CAN ID to set.

    Example:
        set_can_id(bus, 0x01, 0x02)
    """
    data = [0x8B, new_can_id & 0xFF, (new_can_id >> 8) & 0xFF]
    response = send_can_message(bus, old_can_id, data, dlc=4)
    return response


def set_slave_respond_active(bus, can_id, respond, active):
    """
    Set the slave respond and active mode.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.
        respond: 0 (disable respond) or 1 (enable respond).
        active: 0 (disable active) or 1 (enable active).

    Example:
        set_slave_respond_active(bus, 0x01, respond=1, active=1)
    """
    data = [0x8C, respond, active]
    response = send_can_message(bus, can_id, data, dlc=4)
    return response


def set_key_lock(bus, can_id, lock):
    """
    Lock or unlock the key.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.
        lock: 0 (unlock) or 1 (lock).

    Example:
        set_key_lock(bus, 0x01, lock=1)
    """
    data = [0x8F, lock]
    response = send_can_message(bus, can_id, data, dlc=3)
    return response


def set_group_id(bus, can_id, group_id):
    """
    Set the group ID for the motor.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.
        group_id: Group ID to assign (0x01 to 0x7FF).

    Example:
        set_group_id(bus, 0x01, group_id=0x50)
    """
    data = [0x8D, group_id & 0xFF, (group_id >> 8) & 0xFF]
    response = send_can_message(bus, can_id, data, dlc=4)
    return response


def set_home_parameters(bus, can_id, home_trig, home_dir, home_speed, end_limit):
    """
    Set the home parameters for the motor.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.
        home_trig: 0 (low) or 1 (high).
        home_dir: 0 (CW) or 1 (CCW).
        home_speed: Speed in RPM (0-3000).
        end_limit: 0 (disable) or 1 (enable).

    Example:
        set_home_parameters(bus, 0x01, home_trig=1, home_dir=0, home_speed=1000, end_limit=1)
    """
    speed_bytes = [home_speed & 0xFF, (home_speed >> 8) & 0xFF]
    data = [0x90, home_trig, home_dir] + speed_bytes + [end_limit]
    response = send_can_message(bus, can_id, data, dlc=7)
    return response


def go_home(bus, can_id):
    """
    Send the motor to its home position.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.

    Example:
        go_home(bus, 0x01)
    """
    data = [0x91]
    response = send_can_message(bus, can_id, data, dlc=2)
    return response


def set_current_axis_to_zero(bus, can_id):
    """
    Set the current axis to zero.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.

    Example:
        set_current_axis_to_zero(bus, 0x01)
    """
    data = [0x92]
    response = send_can_message(bus, can_id, data, dlc=2)
    return response


def restore_default_parameters(bus, can_id):
    """
    Restore the default parameters of the motor.

    Args:
        bus: CAN bus instance.
        can_id: Motor CAN ID.

    Example:
        restore_default_parameters(bus, 0x01)
    """
    data = [0x3F]
    response = send_can_message(bus, can_id, data, dlc=2)
    return response


def read_encoder_carry_value(bus, motor_id):
    """Read Encoder Carry and Current Value for the given motor ID."""
    command = 0x30
    response = send_can_message(bus, motor_id, [command], dlc=2)
    
    if response and len(response) == 8:
        # Extract carry value (bytes 2-5 as signed int32)
        carry = int.from_bytes(response[1:5], byteorder='little', signed=True)
        # Extract encoder value (bytes 6-7 as unsigned int16)
        value = int.from_bytes(response[5:7], byteorder='little', signed=False)
        return carry, value
    return None, None

def read_combined_encoder_value(bus, motor_id):
    """Read the combined encoder value (carry + value) directly as an integer."""
    command = 0x30
    response = send_can_message(bus, motor_id, [command], dlc=2)
    
    if response and len(response) == 8:
        # Combine bytes 1 through 7 into a single signed integer (48-bit)
        combined_value = int.from_bytes(response[1:7], byteorder='little', signed=True)
        return combined_value
    return None





def read_encoder_addition_value(bus, motor_id):
    """
    Read the encoder addition value for the given motor ID.
    Args:
        bus: The CAN bus instance.
        motor_id: The motor's CAN ID.
    Returns:
        The addition value as an integer if successful, None otherwise.
    """
    command = [0x31]
    response = send_can_message(bus, motor_id, command, dlc=2)
    if response and len(response.data) == 8:
        value = int.from_bytes(response.data[1:7], byteorder='little', signed=True)
        return value
    return None


def read_motor_speed(bus, motor_id):
    """
    Read the real-time speed of the motor in RPM.
    Args:
        bus: The CAN bus instance.
        motor_id: The motor's CAN ID.
    Returns:
        The motor speed in RPM as an integer if successful, None otherwise.
    """
    command = [0x32]
    response = send_can_message(bus, motor_id, command, dlc=2)
    if response and len(response.data) == 4:
        speed = int.from_bytes(response.data[1:3], byteorder='little', signed=True)
        return speed
    return None



def read_pulse_count(bus, motor_id):
    """Read the Number of Pulses Received for the given motor ID."""
    command = 0x33
    return send_can_message(bus, motor_id, [command])


def read_io_ports_status(bus, motor_id):
    """Read IO Ports Status for the given motor ID."""
    command = 0x34
    return send_can_message(bus, motor_id, [command])


def read_motor_shaft_angle_error(bus, motor_id):
    """Read Motor Shaft Angle Error for the given motor ID."""
    command = 0x39
    return send_can_message(bus, motor_id, [command])


def read_en_pin_status(bus, motor_id):
    """Read En Pin Status for the given motor ID."""
    command = 0x3A
    return send_can_message(bus, motor_id, [command])


def read_zero_status(bus, motor_id):
    """Read Go Back to Zero Status for the given motor ID."""
    command = 0x3B
    return send_can_message(bus, motor_id, [command])


def read_locked_rotor_protection(bus, motor_id):
    """Read Locked Rotor Protection State for the given motor ID."""
    command = 0x3D
    return send_can_message(bus, motor_id, [command])


def read_motor_shaft_protection(bus, motor_id):
    """Read Motor Shaft Protection State for the given motor ID."""
    command = 0x3E
    return send_can_message(bus, motor_id, [command])


def query_motor_status(bus, motor_id):
    """
    Queries the motor's current status.

    Command Code: 0xF1

    Args:
        bus: CAN bus interface.
        motor_id: ID of the motor (e.g., 0x01, 0x02).

    Returns:
        Motor status as a byte (e.g., 0 = Fail, 2 = Run Complete).
    
    Example:
        status = query_motor_status(bus, motor_id=4)
    """
    command = 0xF1
    response = send_can_message(bus, motor_id, [command])
    return response



def enable_motor(bus, can_id, enable=True):
    """
    Enable or disable the motor.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        enable: True to enable, False to disable.
    Returns:
        The response status (1 for success, 0 for failure).
    """
    data = [0xF3, 0x01 if enable else 0x00]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def emergency_stop(bus, can_id):
    """
    Perform an emergency stop for the motor.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
    Returns:
        The response status (1 for success, 0 for failure).
    """
    data = [0xF7]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def start_motor_speed_mode(bus, can_id, direction, speed, acc):
    """
    Run the motor in speed mode.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        direction: 0 for CW, 1 for CCW.
        speed: Speed value (0-3000 RPM).
        acc: Acceleration value (0-255).
    Returns:
        The response status (0, 1, or 2 based on documentation).
    """
    speed_high = (speed >> 8) & 0x0F
    speed_low = speed & 0xFF
    data = [0xF6, direction << 7 | speed_high, speed_low, acc]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def stop_motor_speed_mode(bus, can_id, acc=0):
    """
    Stop the motor in speed mode.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        acc: Acceleration value (0 for immediate stop, >0 for deceleration stop).
    Returns:
        The response status (0, 1, or 2 based on documentation).
    """
    data = [0xF6, 0x00, 0x00, acc]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def save_clean_parameter_speed_mode(bus, can_id, state="save"):
    """
    Save or clean parameters in speed mode.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        state: "save" to save parameters, "clean" to clean parameters.
    Returns:
        The response status (1 for success, 0 for failure).
    """
    state_code = 0xC8 if state == "save" else 0xCA
    data = [0xFF, state_code]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def start_motor_position_mode(bus, can_id, direction, speed, acc, pulses):
    """
    Run the motor in position mode.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        direction: 0 for CW, 1 for CCW.
        speed: Speed value (0-3000 RPM).
        acc: Acceleration value (0-255).
        pulses: Number of pulses for relative motion.
    Returns:
        The response status (0, 1, 2, or 3 based on documentation).
    """
    speed_high = (speed >> 8) & 0x0F
    speed_low = speed & 0xFF
    pulses_bytes = pulses.to_bytes(3, byteorder="big", signed=False)
    data = [0xFD, direction << 7 | speed_high, speed_low, acc] + list(pulses_bytes)
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None


def stop_motor_position_mode(bus, can_id, acc=0):
    """
    Stop the motor in position mode.
    Args:
        bus: The CAN bus instance.
        can_id: The motor's CAN ID.
        acc: Acceleration value (0 for immediate stop, >0 for deceleration stop).
    Returns:
        The response status (0, 1, 2, or 3 based on documentation).
    """
    data = [0xFD, 0x00, 0x00, acc, 0x00, 0x00, 0x00]
    msg = can.Message(arbitration_id=can_id, data=data + [calculate_crc(can_id, data)], is_extended_id=False)
    bus.send(msg)
    response = bus.recv(timeout=2)
    if response:
        return response.data[1]
    return None

def start_motor_position_mode2(bus, motor_id, speed, acc, abs_pulses):
    """
    Run the motor in position mode 2 (absolute motion by pulses).

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        speed: Speed in RPM (0-3000).
        acc: Acceleration (0-255).
        abs_pulses: Absolute axis position (-8388607 to +8388607).

    Returns:
        The status from the motor.
    """
    abs_pulses_bytes = abs_pulses.to_bytes(3, byteorder='big', signed=True)
    data = [0xFE, speed, acc] + list(abs_pulses_bytes)
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None
def start_motor_position_mode2(bus, motor_id, speed, acc, abs_pulses):
    """
    Run the motor in position mode 2 (absolute motion by pulses).

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        speed: Speed in RPM (0-3000).
        acc: Acceleration (0-255).
        abs_pulses: Absolute axis position (-8388607 to +8388607).

    Returns:
        The status from the motor.
    """
    abs_pulses_bytes = abs_pulses.to_bytes(3, byteorder='big', signed=True)
    data = [0xFE, speed, acc] + list(abs_pulses_bytes)
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None
def start_motor_position_mode3(bus, motor_id, speed, acc, rel_axis):
    """
    Run the motor in position mode 3 (relative motion by axis).

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        speed: Speed in RPM (0-3000).
        acc: Acceleration (0-255).
        rel_axis: Relative axis position (-8388607 to +8388607).

    Returns:
        The status from the motor.
    """
    rel_axis_bytes = rel_axis.to_bytes(3, byteorder='big', signed=True)
    data = [0xF4, speed, acc] + list(rel_axis_bytes)
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None
def stop_motor_position_mode3(bus, motor_id, acc=0):
    """
    Stop the motor in position mode 3.

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        acc: Acceleration value (0 for immediate stop, >0 for deceleration stop).

    Returns:
        The status from the motor.
    """
    data = [0xF4, 0, acc, 0, 0, 0]
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None
def start_motor_position_mode4(bus, motor_id, speed, acc, abs_axis):
    """
    Run the motor in position mode 4 (absolute motion by axis).

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        speed: Speed in RPM (0-3000).
        acc: Acceleration (0-255).
        abs_axis: Absolute axis position (-8388607 to +8388607).

    Returns:
        The status from the motor.
    """
    abs_axis_bytes = abs_axis.to_bytes(3, byteorder='big', signed=True)
    data = [0xF5, speed, acc] + list(abs_axis_bytes)
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None
def stop_motor_position_mode4(bus, motor_id, acc=0):
    """
    Stop the motor in position mode 4.

    Args:
        bus: CAN bus instance.
        motor_id: Motor CAN ID.
        acc: Acceleration value (0 for immediate stop, >0 for deceleration stop).

    Returns:
        The status from the motor.
    """
    data = [0xF5, 0, acc, 0, 0, 0]
    response = send_can_message(bus, motor_id, data)
    if response:
        return response[1]
    return None

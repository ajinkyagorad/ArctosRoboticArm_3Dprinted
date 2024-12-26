import can
import sys
import time

def calculate_crc(can_id, data):
    """Calculate the 8-bit CRC for the given CAN ID and data."""
    crc_data = [can_id & 0xFF, (can_id >> 8) & 0xFF]  # Include CAN ID (as two bytes)
    crc_data.extend(data)  # Append the message data
    return sum(crc_data) & 0xFF  # Compute the CRC

def send_can_message(bus, can_id, data, timeout=2):
    """Send a CAN message and listen for a reply."""
    # Calculate the CRC
    crc = calculate_crc(can_id, data)

    # Append the CRC to the data
    data.append(crc)

    # Create the CAN message
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)

    try:
        # Send the CAN message
        bus.send(msg)
        print(f"Message sent on {bus.channel_info}: {msg}")
    except can.CanError:
        print("Message NOT sent")
        return

    # Wait for a reply
    print("Waiting for a reply...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout)
        if msg:
            print(f"Reply received: {msg}")
            break
    else:
        print("No reply received within the timeout period.")

if __name__ == "__main__":
    """
    Example usage for Arctos ARM:
    Send multiple CAN messages in a stream:
    python cansend.py 003#FD8F400202F7F0 001#FD8F400202F7F0
    """

    # Check command-line arguments
    if len(sys.argv) < 2:
        print("Usage: python cansend.py <CAN-ID>#<DATA> [<CAN-ID>#<DATA> ...]")
        sys.exit(1)

    # Initialize the CAN bus
    try:
        bus = can.interface.Bus(bustype="slcan", channel="/dev/ttyACM6", bitrate=500000)
    except Exception as e:
        print(f"Error initializing CAN bus: {e}")
        sys.exit(1)

    try:
        # Iterate over all messages provided in the command-line arguments
        for input_command in sys.argv[1:]:
            # Parse the input
            can_id_hex, data_hex = input_command.split('#')

            # Convert CAN ID and data to integers
            can_id = int(can_id_hex, 16)
            data = [int(data_hex[i:i+2], 16) for i in range(0, len(data_hex), 2)]

            # Send the CAN message and wait for a reply
            send_can_message(bus, can_id, data)
    finally:
        # Shutdown the bus
        bus.shutdown()

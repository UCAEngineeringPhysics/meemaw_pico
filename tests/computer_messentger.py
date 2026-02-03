import serial
from time import time, sleep

# SETUP
# Linux example: '/dev/ttyACM0', '/dev/ttyACM1'
# Mac example: '/dev/cu.usbmodem1413301''
# Windows example: 'COM3', 'COM4'
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
# BAUD_RATE = 921600
usb_messenger = serial.Serial(
    SERIAL_PORT, BAUD_RATE, timeout=0.01
)  # set error recovery duration half the pico talker's period
print(f"Connected to {SERIAL_PORT}")
sleep(2)  # Wait briefly for the connection to stabilize
# Variables
tlvs = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
targ_lin_vels = tlvs + list(reversed(tlvs))
msg_id = 0
last_stamp = time()
print("Starting communication... Press Ctrl+C to stop.")

# LOOP
try:
    while True:
        # Transmit data (TX)
        current_stamp = time()
        if (current_stamp - last_stamp) >= 0.5:  # 2Hz TX
            msg = f"{targ_lin_vels[msg_id%16]},0.0\n"
            # Encode string to bytes and send
            usb_messenger.write(msg.encode("utf-8"))
            msg_id += 1
            last_stamp = current_stamp

        # Receive data (RX)
        if usb_messenger.in_waiting > 0:
            try:
                # Read line, decode bytes to string, and strip whitespace
                in_msg = usb_messenger.readline().decode("utf-8").strip()
                if in_msg:
                    print(f"Computer heard: {in_msg}")
                else:
                    print("No message received.")
            except UnicodeDecodeError:
                pass  # ignore decoding errors

except KeyboardInterrupt:
    print("\nCommunication stopped by user.")
    usb_messenger.close()

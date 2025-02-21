import serial
import time
from datetime import datetime

# Configure the serial connection
ser = serial.Serial(
    port="/dev/cu.usbserial-0001",  # Change this to your port (COM3 on Windows)
    baudrate=115200,
    timeout=1,
)

# Create filename with timestamp
filename = f"robot_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Open file and start collecting data
with open(filename, "w") as file:
    print(f"Recording data to {filename}")
    print("Will send START command in 10 seconds...")
    print("Press Ctrl+C to stop...")

    start_time = time.time()
    command_sent = False
    discard_data = True

    try:
        while True:
            # Send START command after 10 seconds
            if not command_sent and (time.time() - start_time >= 2):
                print("Sending START command...")
                ser.write(b"START\n")
                command_sent = True

            if ser.in_waiting:
                line = ser.readline().decode("utf-8").strip()

                # Check for marker to start recording actual data
                if line == "DISCARD_DATA":
                    discard_data = False
                    continue
                elif line == "STOP":
                    break

                # Only save and print data after the marker
                if not discard_data:
                    print(line)  # Print to console
                    file.write(line + "\n")  # Write to file
                    file.flush()  # Ensure data is written immediately
                else:
                    print(f"(Discarded) {line}")  # Show discarded data

    except KeyboardInterrupt:
        print("\nStopping data collection...")
    finally:
        ser.close()
        print("Serial port closed")

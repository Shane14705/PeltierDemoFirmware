import serial #pip install pyserial
import time
import csv
from datetime import datetime

# --- Configuration ---
SERIAL_PORT = '/dev/cu.usbmodem101'      # Change to 'COM3', '/dev/ttyUSB0', etc.
BAUD_RATE = 115200        # Must match ESP32 baud rate
CSV_FILENAME = datetime.now().strftime('%Y-%m-%d_H:%M:%S_') + 'sensor_data.csv'

def start_logging():
    # Open the CSV file in append mode
    # 'newline=""' is required for the csv module to handle line endings correctly
    with open(CSV_FILENAME, mode='a', newline='') as file:
        writer = csv.writer(file)

        # Write the header row if the file is empty
        if file.tell() == 0:
            writer.writerow(["Timestamp", "HapticState", "HeartRate", "Temp1", "Target1", "Temp2", "Target2", "Temp3", "Target3"])

        try:
            # Initialize serial connection
            print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            time.sleep(2) # Allow time for connection to stabilize
            print("Connected! Press Ctrl+C to stop logging.\n")

            while True:
                if ser.in_waiting > 0:
                    # Read line, decode binary to text, strip whitespace
                    line = ser.readline().decode('utf-8', errors='ignore').strip()

                    if line:
                        if (line.startswith("LOG: ")):
                            line = line.lstrip("LOG: ")
                            # Create timestamp
                            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                            # Expected format from ESP32: "123,456,789"
                            data_values = line.split(',')

                            # create a row with timestamp first, then the data
                            row = [timestamp] + data_values

                            # Write to CSV and print to console for feedback
                            writer.writerow(row)
                            print(f"Logged: {row}")

                            # Ensure data is written to disk immediately
                            file.flush()

        except serial.SerialException as e:
            print(f"\nError: Could not open serial port {SERIAL_PORT}.")
            print(f"Details: {e}")
        except KeyboardInterrupt:
            print("\nLogging stopped by user.")
        finally:
            if 'ser' in locals() and ser.is_open:
                ser.close()
                print("Serial connection closed.")

if __name__ == "__main__":
    start_logging()
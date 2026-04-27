#!/usr/bin/env python3
"""
Simple serial monitor for debugging Arduino communication
Useful for troubleshooting the h_motor slider issue
"""

import serial
import serial.tools.list_ports
import sys

def main():
    # Find available ports
    ports = [port.device for port in serial.tools.list_ports.comports()]
    print("Available ports:", ports)

    if not ports:
        print("No serial ports found!")
        return

    port = ports[0]
    baud = 115200

    print(f"Connecting to {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)

    print("Connected. Press Ctrl+C to exit.")
    print("=" * 60)

    try:
        while True:
            # Read from Arduino
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    print(f"<-- {line}")

            # Read from user
            try:
                user_input = input("")
                if user_input:
                    ser.write((user_input + "\n").encode())
                    print(f"--> {user_input}")
            except EOFError:
                pass
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

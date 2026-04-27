#!/usr/bin/env python3
"""
Quick test to verify GUI loads and functions work
"""

import tkinter as tk
from motor_control import MotorControlApp

def test_gui():
    """Test that GUI can load without errors"""
    root = tk.Tk()
    app = MotorControlApp(root)

    # Test that log_message works
    print("Testing log_message method...")
    app.log_message("Test message 1")
    app.log_message("Test message 2")
    app.log_message("Test message 3")

    # Test that slider variables work
    print(f"H angle: {app.h_angle.get()}")
    print(f"V angle: {app.v_angle.get()}")

    # Change slider values
    app.h_angle.set(45.0)
    app.v_angle.set(-30.0)

    print(f"H angle after set: {app.h_angle.get()}")
    print(f"V angle after set: {app.v_angle.get()}")

    # Manually trigger slider change
    print("\nTesting on_slider_change...")
    app.on_slider_change()

    # Process some events
    root.update()

    print("\nGUI test completed successfully!")
    print("Log display visible in window above")

    # Keep window open for 3 seconds then close
    root.after(3000, root.destroy)
    root.mainloop()

if __name__ == "__main__":
    test_gui()

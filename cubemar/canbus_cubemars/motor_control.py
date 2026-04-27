#!/usr/bin/env python3
"""
CubeMars Dual Motor Controller GUI
Controls two motors (h_motor & v_motor) via serial connection to Arduino Nano
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("CubeMars Dual Motor Controller")
        self.root.geometry("700x800")
        self.root.resizable(True, True)

        self.ser = None
        self.running = False
        self.motor_enabled = False
        self.last_motor_response = 0
        self.motor_responding = False

        # Default values
        self.h_angle = tk.DoubleVar(value=0.0)
        self.v_angle = tk.DoubleVar(value=0.0)
        self.kp = tk.DoubleVar(value=1.0)
        self.kd = tk.DoubleVar(value=0.5)
        self.motor_select = tk.StringVar(value="h")

        self.setup_ui()
        self.find_ports()
        self.update_status()  # Update status periodically

    def setup_ui(self):
        """Create the user interface"""

        # ── Connection Frame ──
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=10)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky=tk.W)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, sticky=tk.W)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(conn_frame, textvariable=self.baud_var, values=["9600", "115200", "230400"], width=10, state="readonly").grid(row=0, column=3, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_serial)
        self.connect_btn.grid(row=0, column=4, padx=5)

        self.status_label = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.status_label.grid(row=1, column=0, columnspan=5, sticky=tk.W, pady=5)

        # ── Status Panel ──
        status_frame = ttk.LabelFrame(self.root, text="Status Report", padding=10)
        status_frame.pack(fill=tk.X, padx=10, pady=10)

        status_content = ttk.Frame(status_frame)
        status_content.pack(fill=tk.X)

        ttk.Label(status_content, text="Connection:", font=("Arial", 9)).grid(row=0, column=0, sticky=tk.W, padx=5)
        self.conn_status = ttk.Label(status_content, text="❌ Disconnected", foreground="red", font=("Arial", 9, "bold"))
        self.conn_status.grid(row=0, column=1, sticky=tk.W, padx=5)

        ttk.Label(status_content, text="Motor Enabled:", font=("Arial", 9)).grid(row=0, column=2, sticky=tk.W, padx=5)
        self.motor_enabled_status = ttk.Label(status_content, text="❌ Disabled", foreground="red", font=("Arial", 9, "bold"))
        self.motor_enabled_status.grid(row=0, column=3, sticky=tk.W, padx=5)

        ttk.Label(status_content, text="Motor Responding:", font=("Arial", 9)).grid(row=1, column=0, sticky=tk.W, padx=5)
        self.motor_response_status = ttk.Label(status_content, text="⚪ Waiting...", foreground="gray", font=("Arial", 9, "bold"))
        self.motor_response_status.grid(row=1, column=1, sticky=tk.W, padx=5)

        ttk.Label(status_content, text="Last Response:", font=("Arial", 9)).grid(row=1, column=2, sticky=tk.W, padx=5)
        self.last_response_time = ttk.Label(status_content, text="None", foreground="gray", font=("Arial", 9))
        self.last_response_time.grid(row=1, column=3, sticky=tk.W, padx=5)

        # ── Motor Enable/Disable Frame ──
        control_frame = ttk.LabelFrame(self.root, text="Motor Control", padding=10)
        control_frame.pack(fill=tk.X, padx=10, pady=10)

        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, pady=10)

        self.enable_btn = ttk.Button(btn_frame, text="🟢 Enable Motors", command=self.enable_motors, state=tk.DISABLED)
        self.enable_btn.pack(side=tk.LEFT, padx=5)

        self.disable_btn = ttk.Button(btn_frame, text="🔴 Disable Motors", command=self.disable_motors, state=tk.DISABLED)
        self.disable_btn.pack(side=tk.LEFT, padx=5)

        self.motor_status = ttk.Label(control_frame, text="Motors: DISABLED", foreground="red", font=("Arial", 10, "bold"))
        self.motor_status.pack(fill=tk.X, pady=5)

        # ── PID Tuning Frame ──
        pid_frame = ttk.LabelFrame(self.root, text="PID Tuning", padding=10)
        pid_frame.pack(fill=tk.X, padx=10, pady=10)

        # Motor selection
        motor_select_frame = ttk.Frame(pid_frame)
        motor_select_frame.pack(fill=tk.X, pady=(0, 10))
        ttk.Label(motor_select_frame, text="Motor:").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(motor_select_frame, text="H-Motor", variable=self.motor_select, value="h").pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(motor_select_frame, text="V-Motor", variable=self.motor_select, value="v").pack(side=tk.LEFT, padx=5)

        # Kp slider
        kp_frame = ttk.Frame(pid_frame)
        kp_frame.pack(fill=tk.X, pady=5)
        ttk.Label(kp_frame, text="Kp (0.0-500.0):", width=15).pack(side=tk.LEFT)
        self.kp_slider = ttk.Scale(kp_frame, from_=0.0, to=500.0, orient=tk.HORIZONTAL, variable=self.kp, command=self.on_pid_change)
        self.kp_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.kp_label = ttk.Label(kp_frame, text="1.0", width=8, foreground="blue")
        self.kp_label.pack(side=tk.LEFT)

        # Kd slider
        kd_frame = ttk.Frame(pid_frame)
        kd_frame.pack(fill=tk.X, pady=5)
        ttk.Label(kd_frame, text="Kd (0.0-5.0):", width=15).pack(side=tk.LEFT)
        self.kd_slider = ttk.Scale(kd_frame, from_=0.0, to=5.0, orient=tk.HORIZONTAL, variable=self.kd, command=self.on_pid_change)
        self.kd_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        self.kd_label = ttk.Label(kd_frame, text="0.5", width=8, foreground="green")
        self.kd_label.pack(side=tk.LEFT)

        # Set PID button
        ttk.Button(pid_frame, text="Send PID Command", command=self.send_pid).pack(pady=5)

        # ── H-Motor Slider ──
        h_frame = ttk.LabelFrame(self.root, text="H-Motor (Horizontal)", padding=10)
        h_frame.pack(fill=tk.X, padx=10, pady=10)

        slider_frame = ttk.Frame(h_frame)
        slider_frame.pack(fill=tk.X)

        ttk.Label(slider_frame, text="-180°").pack(side=tk.LEFT)
        self.h_slider = ttk.Scale(slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL,
                                   variable=self.h_angle, command=self.on_slider_change)
        self.h_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(slider_frame, text="180°").pack(side=tk.LEFT)

        self.h_value_label = ttk.Label(h_frame, text="0.0°", font=("Arial", 16, "bold"), foreground="blue")
        self.h_value_label.pack(fill=tk.X, pady=5)

        # ── V-Motor Slider ──
        v_frame = ttk.LabelFrame(self.root, text="V-Motor (Vertical)", padding=10)
        v_frame.pack(fill=tk.X, padx=10, pady=10)

        slider_frame = ttk.Frame(v_frame)
        slider_frame.pack(fill=tk.X)

        ttk.Label(slider_frame, text="-180°").pack(side=tk.LEFT)
        self.v_slider = ttk.Scale(slider_frame, from_=-180, to=180, orient=tk.HORIZONTAL,
                                   variable=self.v_angle, command=self.on_slider_change)
        self.v_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Label(slider_frame, text="180°").pack(side=tk.LEFT)

        self.v_value_label = ttk.Label(v_frame, text="0.0°", font=("Arial", 16, "bold"), foreground="green")
        self.v_value_label.pack(fill=tk.X, pady=5)

        # ── Quick Presets ──
        preset_frame = ttk.LabelFrame(self.root, text="Quick Presets", padding=10)
        preset_frame.pack(fill=tk.X, padx=10, pady=10)

        preset_btn_frame = ttk.Frame(preset_frame)
        preset_btn_frame.pack(fill=tk.X)

        ttk.Button(preset_btn_frame, text="Center (0°, 0°)", command=lambda: self.set_angles(0, 0)).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(preset_btn_frame, text="Home (-90°, -90°)", command=lambda: self.set_angles(-90, -90)).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)
        ttk.Button(preset_btn_frame, text="Up (0°, 90°)", command=lambda: self.set_angles(0, 90)).pack(side=tk.LEFT, padx=2, fill=tk.X, expand=True)

        # ── Serial Log Frame ──
        log_frame = ttk.LabelFrame(self.root, text="Serial Communication Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create text widget with scrollbar
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL)
        self.info_text = tk.Text(log_frame, height=8, width=80, state=tk.DISABLED, yscrollcommand=scrollbar.set, font=("Courier", 9))
        scrollbar.config(command=self.info_text.yview)

        self.info_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.log_message("Ready. Connect to Arduino and enable motors.")

    def find_ports(self):
        """Find available serial ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports if ports else ["No ports found"]
        if ports:
            self.port_combo.current(0)

    def log_message(self, msg):
        """Add message to info text"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.info_text.config(state=tk.NORMAL)
        self.info_text.insert(tk.END, f"[{timestamp}] {msg}\n")
        self.info_text.see(tk.END)
        self.info_text.config(state=tk.DISABLED)

    def connect_serial(self):
        """Connect to Arduino serial port"""
        if self.ser and self.ser.is_open:
            self.disconnect_serial()
            return

        port = self.port_var.get()
        baud = int(self.baud_var.get())

        if not port or "No ports" in port:
            messagebox.showerror("Error", "No serial port selected")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.running = True
            self.status_label.config(text=f"Connected: {port}", foreground="green")
            self.connect_btn.config(text="Disconnect")
            self.enable_btn.config(state=tk.NORMAL)
            self.disable_btn.config(state=tk.NORMAL)
            self.log_message(f"✓ Connected to {port} at {baud} baud")

            # Set default PID on connection
            time.sleep(0.5)
            self.send_command("pid,h,1.0,0.5")
            self.log_message("→ Setting default PID: h_motor kp=1.0 kd=0.5")

            # Start reading thread
            self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
            self.read_thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self.log_message(f"✗ Connection failed: {e}", "error")

    def disconnect_serial(self):
        """Disconnect from Arduino"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.status_label.config(text="Disconnected", foreground="red")
        self.connect_btn.config(text="Connect")
        self.enable_btn.config(state=tk.DISABLED)
        self.disable_btn.config(state=tk.DISABLED)
        self.motor_status.config(text="Motors: DISABLED", foreground="red")
        self.conn_status.config(text="❌ Disconnected", foreground="red")
        self.motor_response_status.config(text="⚪ Waiting...", foreground="gray")
        self.last_response_time.config(text="None")
        self.log_message("✓ Disconnected")

    def send_command(self, cmd):
        """Send command to Arduino"""
        if not self.ser or not self.ser.is_open:
            messagebox.showerror("Error", "Not connected to Arduino")
            return False
        try:
            self.ser.write((cmd + "\n").encode())
            self.log_message(f"→ [SEND] {cmd}")
            return True
        except Exception as e:
            self.log_message(f"✗ Send error: {e}")
            return False

    def read_serial(self):
        """Read responses from Arduino (runs in background thread)"""
        while self.running:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode().strip()
                    if line:
                        self.log_message(f"← [RECV] {line}")

                        # Track motor responses (any line starting with "h_motor" or containing "MIT")
                        if "h_motor" in line or "MIT cmd" in line or ">>" in line:
                            self.last_motor_response = time.time()
                            self.motor_responding = True
            except:
                pass
            time.sleep(0.1)

    def enable_motors(self):
        """Send enable command"""
        if self.send_command("enable"):
            self.motor_enabled = True
            self.motor_status.config(text="Motors: ENABLED ✓", foreground="green")
            self.motor_enabled_status.config(text="✅ Enabled", foreground="green")
            self.log_message("[LOCAL] Motor enabled flag set to True")

    def disable_motors(self):
        """Send disable command"""
        if self.send_command("disable"):
            self.motor_enabled = False
            self.motor_status.config(text="Motors: DISABLED", foreground="red")
            self.motor_enabled_status.config(text="❌ Disabled", foreground="red")
            self.log_message("[LOCAL] Motor enabled flag set to False")

    def on_slider_change(self, value=None):
        """Update angle values and send move command"""
        h = self.h_angle.get()
        v = self.v_angle.get()

        self.h_value_label.config(text=f"{h:.1f}°")
        self.v_value_label.config(text=f"{v:.1f}°")

        # Only send command if motors are enabled and connected
        if self.motor_enabled and self.ser and self.ser.is_open:
            cmd = f"move,{h:.1f},{v:.1f}"
            self.send_command(cmd)

    def set_angles(self, h, v):
        """Set both angle sliders and send command"""
        self.h_angle.set(h)
        self.v_angle.set(v)
        self.on_slider_change()

    def on_pid_change(self, value=None):
        """Update PID labels when sliders change"""
        kp = self.kp.get()
        kd = self.kd.get()
        self.kp_label.config(text=f"{kp:.2f}")
        self.kd_label.config(text=f"{kd:.3f}")

    def send_pid(self):
        """Send PID command to Arduino"""
        kp = self.kp.get()
        kd = self.kd.get()
        motor = self.motor_select.get()

        cmd = f"pid,{motor},{kp:.2f},{kd:.3f}"
        self.log_message(f"[LOCAL] Sending PID: {cmd}")

        if self.ser and self.ser.is_open:
            self.send_command(cmd)
        else:
            self.log_message("✗ Not connected to Arduino")

    def update_status(self):
        """Update motor response status periodically"""
        if self.motor_responding and self.last_motor_response > 0:
            elapsed = time.time() - self.last_motor_response
            if elapsed < 2:  # Response within last 2 seconds
                self.motor_response_status.config(text="✅ Responding", foreground="green")
                from datetime import datetime
                self.last_response_time.config(text=datetime.fromtimestamp(self.last_motor_response).strftime("%H:%M:%S"))
            else:
                self.motor_response_status.config(text="⚠️ No recent response", foreground="orange")
        else:
            self.motor_response_status.config(text="⚪ Waiting...", foreground="gray")

        # Schedule next update
        self.root.after(1000, self.update_status)

def main():
    root = tk.Tk()
    app = MotorControlApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()

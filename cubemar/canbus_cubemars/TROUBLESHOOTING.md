# H-Motor Slider Troubleshooting Guide

## Problem
The h_motor slider in the Python GUI doesn't cause the motor to move when adjusted.

## Debug Steps

### Step 1: Upload Arduino Code
Make sure you upload the latest `canbus_cubemars.ino` with debug output enabled.

### Step 2: Test with Serial Monitor
Use the simple serial monitor tool or Arduino IDE Serial Monitor at 115200 baud:

```bash
python serial_monitor.py
```

### Step 3: Expected Output Sequence

When you connect and test:

1. **Connection establishment:**
   ```
   MCP2515 ready
   Scanning for CubeMars motors...
   ... (motor scan output)
   1 motor(s) found.
   ⚠ Motors disabled at startup for safety.
   ```

2. **Send "enable" command:**
   - Type: `enable`
   - Expected output:
   ```
   DEBUG: motorsEnabled set to 1
   Motors ENABLED - be careful!
   >> enable motor ID 104
   ```

3. **Send "move,45,0" command (move h_motor to 45°):**
   - Type: `move,45,0`
   - Expected output:
   ```
   DEBUG: move cmd parsed a0=45.00 a1=0.00
   DEBUG: Set targetAngles[0]=45.00
   ok,45.00,0.00
   DEBUG: targetAngles[0]=45.00
   >> MIT cmd h_motor pos=0.79 kp=1.0 kd=0.50
   ```

   If you DON'T see the "DEBUG: move cmd parsed..." message, the move command wasn't received by Arduino.

### Step 4: Use Python GUI with Debug

1. Run the GUI: `python motor_control.py`
2. Connect to port (click "Connect")
3. Watch the log window for DEBUG messages
4. Click "Enable Motors"
   - Should see: `Motors ENABLED - be careful!`
5. Move the h_motor slider
   - Should see: `[DEBUG] on_slider_change: h=XX.X, v=0.0, motor_enabled=True, ser_open=True`
   - Should see: `→ move,XX.X,0.0`
   - Arduino should respond with: `DEBUG: move cmd parsed...`

## Diagnosis

### If you DON'T see "[DEBUG] on_slider_change" when moving slider:
- **Problem:** Slider is not triggering the callback
- **Solution:** Check Python code, try dragging slider more deliberately

### If you see "[DEBUG] on_slider_change" but motor_enabled=False:
- **Problem:** Enable button didn't set the flag
- **Solution:** Check that you clicked "Enable Motors" and saw the status change

### If you see "[DEBUG] on_slider_change" with motor_enabled=True but no "→ move" message:
- **Problem:** Serial connection is closed
- **Solution:** Check serial connection status, reconnect

### If you see "→ move,XX,0.0" in Python but NO "DEBUG: move cmd parsed" in Arduino:
- **Problem:** Arduino not receiving the command
- **Possible causes:**
  - USB cable disconnected
  - Wrong COM port selected
  - Baud rate mismatch
  - Arduino not actually programmed with the latest code
- **Solution:** 
  - Check USB connection
  - Test with serial_monitor.py to confirm connection works
  - Re-upload Arduino code
  - Try different USB port

### If you see "DEBUG: move cmd parsed" but motor doesn't move:
- **Problem:** Arduino receives command but motor doesn't move
- **Solution:** Check with "v" command to enable verbose mode, verify motor is responding

## Advanced Testing

### Manual command testing (Arduino Serial Monitor):
```
enable           → Motors should enable
move,45,0        → h_motor should move to 45°
move,-45,0       → h_motor should move to -45°
v                → Enable verbose mode to see motor status
```

### Checking motor response:
```
enable
v
move,30,0        → Should see motor status updates
```

Motor status should look like:
```
h_motor angle=30.0 deg  speed=0.0 RPM  current=0 mA  temp=0 C  err=0x00
```

## Common Issues & Solutions

| Issue | Check | Solution |
|-------|-------|----------|
| No motors found | Scan output | Power on motors, check CAN wiring |
| Motors found but won't enable | Serial output | Try `enable` in Serial Monitor |
| Enable works but motor won't move | `move,45,0` in Serial Monitor | Motor might be in wrong mode, try `mit` command |
| Slider works in Serial Monitor but not GUI | `[DEBUG]` output in Python | Serial connection might be closing |
| Motor moves very slowly | Default Kp/Kd | Adjust with `pid,h,X,Y` command in Serial Monitor |

## Getting More Help

If the debug output doesn't match expectations above, share:
1. Full Arduino Serial Monitor output after connecting
2. Python GUI log after moving slider
3. Whether other commands work (enable/disable via Serial Monitor)

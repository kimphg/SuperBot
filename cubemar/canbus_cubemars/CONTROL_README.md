# CubeMars Dual Motor Controller - Python GUI

A user-friendly GUI application to control two CubeMars motors (h_motor and v_motor) via Arduino Nano.

## Features

- ✅ Two independent angle sliders (-180° to +180°)
- ✅ Enable/Disable motor controls with buttons
- ✅ Quick preset buttons (Center, Home, Up)
- ✅ Real-time angle display
- ✅ Serial communication with Arduino
- ✅ Live command/response logging
- ✅ Safe operation (motors disabled at startup)

## Requirements

```bash
pip install pyserial
```

## Installation & Usage

### 1. Upload Arduino Sketch

Upload `canbus_cubemars.ino` to your Arduino Nano with:
- MCP2515 CAN module on SPI (CS pin 10)
- Two CubeMars motors in MIT Cheetah mode

### 2. Run Python App

```bash
python motor_control.py
```

Or directly:
```bash
./motor_control.py
```

### 3. Connect & Operate

1. **Connect**: Select serial port → Click "Connect" (motors still disabled)
2. **Enable**: Click "🟢 Enable Motors" (motors now active, holding position)
3. **Control**: 
   - Move slider for h_motor (horizontal) or v_motor (vertical)
   - Sliders send `move,<h_angle>,<v_angle>` command automatically
4. **Presets**: Quick buttons for Center, Home, Up positions
5. **Disable**: Click "🔴 Disable Motors" when done

## GUI Layout

```
┌─────────────────────────────────────────┐
│         Serial Connection               │
│  Port: [COM3▼] Baud: 115200▼ [Connect]│
│  Status: Connected (green)              │
├─────────────────────────────────────────┤
│         Motor Control                   │
│  [🟢 Enable]  [🔴 Disable]             │
│  Motors: ENABLED ✓                      │
├─────────────────────────────────────────┤
│    H-Motor (Horizontal)                 │
│  -180° ═════●═════ +180°                │
│  Current: 0.0°                          │
├─────────────────────────────────────────┤
│    V-Motor (Vertical)                   │
│  -180° ═════●═════ +180°                │
│  Current: 0.0°                          │
├─────────────────────────────────────────┤
│         Quick Presets                   │
│  [Center] [Home] [Up]                   │
├─────────────────────────────────────────┤
│         Info (Log)                      │
│  ✓ Connected to COM3 at 115200 baud    │
│  → Command: enable                      │
│  → move,0.0,0.0                         │
│  >> enable motor ID 104                 │
│  >> MIT cmd h_motor pos=0.00 kp=20.0   │
└─────────────────────────────────────────┘
```

## Serial Commands Sent

The app sends these commands via serial:

- `enable` — Activate motors (required before moving)
- `disable` — Deactivate motors (safe at any time)
- `move,<h_angle>,<v_angle>` — Set target angles (only works when enabled)
- Automatically reads and logs Arduino responses

## Angle Limits

- **h_motor (AK60-6)**: -180° to +180°
- **v_motor (AK45-36)**: -180° to +180°

Actual motor ranges may be smaller based on physical configuration.

## Troubleshooting

### No ports appear
- Check USB cable connection
- Verify Arduino drivers installed (`CH340` for cheap Nano clones)
- Try different USB port on computer

### "Connection Error"
- Make sure Arduino is programmed with `canbus_cubemars.ino`
- Check baud rate matches (default 115200)
- Click "Disconnect" first, then reconnect

### Motors don't move after Enable
- Check Arduino serial output (open Arduino IDE Serial Monitor)
- Verify CAN wiring to MCP2515
- Try sending `mit` command in Arduino Serial Monitor to re-switch motors to MIT mode

### Motors move when I don't want them to
- Motors are only active when "Motors: ENABLED ✓" is shown
- Click "🔴 Disable Motors" to stop them
- Check that sliders aren't accidentally being moved

## Motor Status (Optional)

To see real-time motor status (speed, current, temp):
1. Click "Enable Motors"
2. Open Arduino Serial Monitor at 115200 baud
3. Type `v` to enable verbose mode
4. You'll see status updates like:
   ```
   >> MIT cmd h_motor pos=0.00 kp=20.0 kd=0.50
   h_motor angle=0.0 deg  speed=0.0 RPM  current=0 mA  temp=0 C  err=0x00
   ```

## Advanced: Custom PID Tuning

For fine-tuning motor responsiveness, open Arduino Serial Monitor and send:

```
pid,h,20.0,0.5     # Set h_motor: kp=20.0, kd=0.5
pid,v,20.0,1.0     # Set v_motor: kp=20.0, kd=1.0
```

Lower `kp` = slower/smoother response  
Higher `kd` = more damping (less oscillation)

## Safety Notes

⚠️ **Always:**
- Start with motors **DISABLED**
- Click Enable only when you're ready and the area is clear
- Keep hands away from motor during movement
- Click Disable immediately if movement is erratic
- Never hold Enable longer than needed

## License

MIT License - Feel free to modify for your use case.

# CubeMars Dual Motor Controller

Controls two CubeMars motors (h\_motor + v\_motor) via Arduino Nano + MCP2515 CAN module, using MIT Cheetah Mode protocol.

## Hardware

| Part | Details |
|------|---------|
| Microcontroller | Arduino Nano (AVR, 16-bit int) |
| CAN module | MCP2515, CS pin 10, 8 MHz crystal, 1 Mbps |
| h\_motor | AK60-6 V1.1, CAN ID 104 |
| v\_motor | AK45-36, CAN ID 111 |

## Files

| File | Purpose |
|------|---------|
| `canbus_cubemars.ino` | Arduino firmware (MIT Cheetah mode) |
| `motor_control.py` | Python GUI controller |
| `serial_monitor.py` | Simple serial terminal for manual testing |
| `MIT_MODE_ARCHIVE.h` | MIT mode code archive (not included by default) |
| `MIT_MODE_USAGE.md` | How to re-enable MIT mode archive |

## Quick Start

### 1. Upload firmware
Open `canbus_cubemars.ino` in Arduino IDE and upload to Arduino Nano.

### 2. Run GUI
```bash
pip install pyserial
python motor_control.py
```

### 3. Operate
1. Select serial port → **Connect**
2. **Enable Motors** (sends safe Kp=0.5, Kd=0.5 automatically)
3. Drag sliders to move motors
4. **Disable Motors** when done

## Serial Commands (Arduino Serial Monitor at 115200 baud)

| Command | Effect |
|---------|--------|
| `enable` | Enable all motors |
| `disable` | Disable all motors |
| `move,<h>,<v>` | Set target angles in degrees (e.g. `move,45,0`) |
| `pid,<h\|v>,<kp>,<kd>` | Tune gains (e.g. `pid,h,5.0,0.5`) |
| `status` | Read position/velocity/current from all motors |
| `status,h` or `status,v` | Read single motor |
| `listen` | Listen 5 s for any raw CAN frames |
| `v` | Toggle verbose mode (raw byte logging) |
| `test` | Ping — replies `Serial OK` |

## MIT Cheetah Mode Protocol

Standard 11-bit CAN frame, ID = motor ID, 8 bytes:

```
Byte 0: Position [15:8]       (16-bit, -12.5 to +12.5 rad)
Byte 1: Position [7:0]
Byte 2: Velocity [11:4]       (12-bit, -30 to +30 rad/s)
Byte 3: Velocity [3:0] | Kp [11:8]  (12-bit, 0-500)
Byte 4: Kp [7:0]
Byte 5: Kd [11:4]             (12-bit, 0-5)
Byte 6: Kd [3:0] | Iff [11:8] (12-bit, ±18 A for AK60, ±30 A for AK45)
Byte 7: Iff [7:0]
```

Special commands (byte 7 only):
- `0xFC` = Enable
- `0xFD` = Disable
- `0xFE` = Set zero

## Critical: AVR Integer Overflow Bug

**The official CubeMars `float_to_uint` sample code uses `int`, which is 32-bit on ARM (their target) but 16-bit on Arduino Nano AVR (max 32767).** The 16-bit position encoding returns values up to 65535, causing silent overflow.

**Symptom:** motor responds to `enable` but ignores all position commands; verbose output shows `p=0 bytes: 00 00 ...` regardless of target.

**Fix applied in this firmware:**
```cpp
// WRONG (official sample, works on ARM only):
static int float_to_uint(float x, float xMin, float xMax, int bits) {
  return (int)((x - xMin) * ((float)((1 << bits) / span)));
}

// CORRECT (AVR-safe):
static uint32_t floatToUintMIT(float x, float xMin, float xMax, uint8_t bits) {
  float span = xMax - xMin;
  if (x < xMin) x = xMin;
  else if (x > xMax) x = xMax;
  return (uint32_t)((x - xMin) * ((float)((1UL << bits) / span)));
}
```

Key changes: return type `uint32_t`, shift uses `1UL << bits` (forces 32-bit math), explicit clamping to `0xFFFF`/`0xFFF`.

## PID Tuning

Kp (stiffness) and Kd (damping) are set per motor. The GUI defaults to Kp=0.5, Kd=0.5 for safety on enable.

| Parameter | Range | Safe start | Effect |
|-----------|-------|-----------|--------|
| Kp | 0–500 | 0.5 | Higher = stiffer, holds position harder |
| Kd | 0–5 | 0.5 | Higher = more damping, less oscillation |

Increase Kp gradually once motion looks stable. Oscillation = reduce Kp or increase Kd.

## Velocity Range

Velocity encoding is fixed at **±30 rad/s** per the CubeMars CAN encoding standard, regardless of the motor's physical maximum speed. Do not change `vMin`/`vMax` from ±30.

## Motor Scan

On startup the firmware scans for motors in two phases:
1. **Passive (500 ms):** listens for any CAN frames (works if motors are already broadcasting)
2. **Active probe:** sends enable to each ID 1–127, listens for reply

Found IDs are assigned as `motorIds[0]` (h\_motor) and `motorIds[1]` (v\_motor) in order of discovery.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| No motors found | CAN wiring / power | Check CANH/CANL wires, motor power |
| Motor enables (moves to 0°) but ignores move commands | AVR int overflow | Confirm firmware uses `uint32_t` in `floatToUintMIT` |
| `p=0` in verbose output | Same overflow bug | Same fix |
| Motor vibrates / oscillates | Kp too high | Lower Kp via `pid,h,X,Y` |
| GUI sends move but nothing happens | Motors not enabled | Click Enable first, or send `enable` in serial monitor |
| `move,100,0` parsed as `a0=0.00` | RAM exhaustion (String heap) | Reduce RAM use; current firmware uses F() macro throughout |

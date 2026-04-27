# MIT Mode Archive - How to Use

The MIT mode code was removed from the main sketch to save memory on Arduino Nano (which was running at 93% capacity).

MIT mode is now archived in `MIT_MODE_ARCHIVE.h` for future use if needed.

## When to Use MIT Mode

MIT mode might be better if:
- Servo mode is not responsive enough
- You need precise velocity control
- You need impedance control (spring-like behavior)
- You need to tune Kp/Kd gains dynamically

## How to Re-enable MIT Mode

### Step 1: Uncomment the include at the top of the main sketch
Add this line after the other #include statements:
```cpp
#include "MIT_MODE_ARCHIVE.h"
```

### Step 2: Add motor name array (needed for MIT mode)
Add this after the motorCount variable declarations:
```cpp
static const char* motorName[2] = { "h_motor", "v_motor" };
```

### Step 3: Change servo to MIT in main loop
In the main loop control section, replace:
```cpp
sendServoPositionCommand(motorIds[i], slewAngles[i]);
```

With:
```cpp
// MIT mode: Calculate desired velocity towards target
float targetVel = 0.0f;
float errorDeg = targetAngles[i] - slewAngles[i];
if (errorDeg > 0.1f) targetVel = 2.0f;
else if (errorDeg < -0.1f) targetVel = -2.0f;
sendMITCommand(motorIds[i], i, slewAngles[i] * DEG_TO_RAD, targetVel * DEG_TO_RAD, 0.0f);
```

### Step 4: Re-add PID tuning command
Add this back to the processSerial() function:
```cpp
} else if (cmd == "pid") {
  // format: pid,<h|v>,<kp>,<kd>
  int c2 = line.indexOf(',', c1 + 1);
  int c3 = line.indexOf(',', c2 + 1);
  if (c2 < 0 || c3 < 0) { Serial.println("err"); return; }
  String motor = line.substring(c1 + 1, c2);
  motor.toLowerCase();
  int idx = motor == "h" ? 0 : motor == "v" ? 1 : -1;
  if (idx < 0 || idx >= (int)motorCount) { Serial.println("err"); return; }
  mitCfg[idx].kp = constrain(line.substring(c2 + 1, c3).toFloat(), MIT_KP_MIN, MIT_KP_MAX);
  mitCfg[idx].kd = constrain(line.substring(c3 + 1).toFloat(),     MIT_KD_MIN, MIT_KD_MAX);
  Serial.print("pid,"); Serial.print(motorName[idx]);
  Serial.print(",kp="); Serial.print(mitCfg[idx].kp, 2);
  Serial.print(",kd="); Serial.println(mitCfg[idx].kd, 3);
```

### Step 5: Recompile and upload

## MIT Mode Commands

Once re-enabled, use these commands:

```
enable              # Enable motors (MIT mode)
move,45,0           # Move h_motor to 45°, v_motor to 0°
pid,h,100,2         # Tune h_motor: Kp=100, Kd=2
pid,v,50,1          # Tune v_motor: Kp=50, Kd=1
disable             # Disable motors
```

## MIT Mode Ranges

- **Position**: ±12.5 radians (±720°)
- **Velocity**: ±30 rad/s (fixed scale in CAN encoding)
- **Kp (stiffness)**: 0-500
- **Kd (damping)**: 0-5
- **Torque feedforward**: ±18 A (AK60-6) or ±30 A (AK45-36)

## Memory Impact

Re-enabling MIT mode will:
- Add ~500 bytes to program storage
- Add ~50 bytes to dynamic memory
- Reduce available RAM to ~40-50% (still safe)

## Known Issues

MIT mode had reliability issues with position control on AK60 V1.1. Servo mode is more stable. Only switch back if you have specific requirements.

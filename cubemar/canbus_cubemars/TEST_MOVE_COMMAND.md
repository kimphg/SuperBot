# Testing the Move Command

Added detailed debug output to help diagnose why move,x,y command isn't working.

## Step 1: Upload Updated Arduino Code

Make sure to upload the latest `canbus_cubemars.ino` with debug output.

## Step 2: Test with Serial Monitor

Open Arduino IDE Serial Monitor at 115200 baud and test manually:

```
enable
```

Expected output:
```
[DEBUG] processSerial got: 'enable'
Motors ENABLED - be careful!
>> enable motor ID 104
```

## Step 3: Send Move Command

In Serial Monitor, type:
```
move,45,0
```

You should see:
```
[DEBUG] processSerial got: 'move,45,0'
[DEBUG] Got 'move' command
[DEBUG] move parsed: a0=45.00 a1=0.00 motorsEnabled=1
[DEBUG] Set targetAngles[0]=45.00
ok,45.00,0.00
```

Then every second, you should see:
```
>> MIT cmd h_motor target=45.00 slew=0.XX pos=0.XX kp=1.0 kd=0.50
```

The slew angle should increase from 0 to 45 gradually (with new slew rates: ~0.45 seconds total).

## Step 4: Check Each Part

### If you DON'T see `[DEBUG] processSerial got: 'move,45,0'`
- **Problem**: Arduino not receiving the command
- **Solution**: Check serial cable, try different USB port, verify baud rate is 115200

### If you see the debug but NOT `[DEBUG] Got 'move' command`
- **Problem**: Command string doesn't match "move"
- **Solution**: Check the command format (spaces, quotes, etc.)

### If you see `ok,45.00,0.00` but motor doesn't move
- **Problem**: MIT command not being sent or motor not responding to CAN
- **Solution**: Check `>> MIT cmd h_motor` output. If it shows `target=0.00`, targetAngles wasn't set

### If slew angle is stuck at 0.XX
- **Problem**: Motor moving but very slowly
- **Solution**: This is normal - watch for several seconds as the angle approaches target

## Step 5: Test with Python GUI

After confirming move command works via Serial Monitor:

1. Run `python motor_control.py`
2. Connect to port
3. Click "Enable Motors"
4. Move h_motor slider to 45°
5. Check Serial Monitor output for debug messages

You should see in the log:
```
[HH:MM:SS] → [SEND] move,45.0,0.0
[HH:MM:SS] ← [RECV] ok,45.00,0.00
```

## Quick Checklist

- [ ] Arduino uploaded with new debug code
- [ ] Serial cable connected
- [ ] Correct COM port selected
- [ ] Baud rate 115200
- [ ] Motors responding to "enable" command
- [ ] Debug output shows command being received
- [ ] Debug output shows targetAngles being set
- [ ] MIT commands being sent every 20ms
- [ ] Motor physically moving (slowly at first)

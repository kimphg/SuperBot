# Debug Raw CAN Bytes - Move Command Issue

This guide helps identify if the move command is encoding bytes incorrectly.

## Key Insight

The **enable command** works and moves the motor to 0°. It sends hardcoded bytes:
```
FF FF FF FF FF FF FF FC
```

The **move command** should send encoded bytes with your target position. If it doesn't work, the bytes might be encoded incorrectly.

## Test Procedure

### Step 1: Upload Latest Code
The code now includes detailed byte logging. Upload it to your Arduino Nano.

### Step 2: Test Enable Command (Working Baseline)
Open Serial Monitor at 115200 baud and type:
```
v
```
This enables verbose mode. You should see:
```
verbose: ON
```

Now send:
```
enable
```

You should see something like:
```
>> enable motor ID 104
[ENABLE RAW] bytes: FF FF FF FF FF FF FF FC
```

This is the WORKING command - it moves the motor to 0°. **Note these exact bytes.**

### Step 3: Test Move Command 
In the same Serial Monitor, type:
```
move,30,0
```

You should see:
```
[DEBUG] processSerial got: 'move,30,0'
[DEBUG] Got 'move' command
[DEBUG] move parsed: a0=30.00 a1=0.00 motorsEnabled=1
[DEBUG] Set targetAngles[0]=30.00
ok,30.00,0.00
```

Then every 20ms, you should see MIT commands being sent:
```
[MIT CMD RAW] ID=104 bytes: XX XX XX XX XX XX XX XX (p=12345 v=2050 kp=41 kd=410 iff=2048)
```

**Copy these exact bytes and share them.** They will look different from the enable command bytes.

### Step 4: Analyze the Encoding

The MIT command should encode like this:

For **move,30,0** (30° = 0.524 rad):
- Position (16 bits): 0.524 rad → should encode to approximately 0x8B2A (35626 in decimal)
- Velocity (12 bits): 2.0 rad/s → should encode to approximately 0x8DD (2269)
- Kp (12 bits): 5.0 → should encode to approximately 0x029 (41)
- Kd (12 bits): 0.5 → should encode to approximately 0x19A (410)
- Iff (12 bits): 0.0 → should encode to approximately 0x800 (2048)

The byte packing for MIT frame:
```
Byte 0: [Position bits 15-8]
Byte 1: [Position bits 7-0]
Byte 2: [Velocity bits 11-4]
Byte 3: [Velocity bits 3-0 << 4 | Kp bits 11-8]
Byte 4: [Kp bits 7-0]
Byte 5: [Kd bits 11-4]
Byte 6: [Kd bits 3-0 << 4 | Iff bits 11-8]
Byte 7: [Iff bits 7-0]
```

## Expected Results

### If Motor Moves After Move Command
Then the bytes are being encoded correctly! The issue might be:
- Different V1.1 protocol that works differently
- Motor needs multiple commands to start moving
- Something else in the control flow

### If Motor Does NOT Move After Move Command
Check:
1. **Are the bytes completely wrong?** 
   - If they're all FF FF FF FF FF FF FF XX, then targetAngles isn't being set
   - Check that motorsEnabled is showing as 1

2. **Are the bytes close to expected but slightly off?**
   - Could be byte packing issue in sendMITCommand
   - Check the bit shifting logic

3. **Are the bytes sent at all?**
   - If you don't see any [MIT CMD RAW] output after the move command, the main loop isn't sending them
   - Check that motorsEnabled is true

## Bonus Test: Pure Position Control

If the move command still doesn't work, test with velocity=0:

```
testmove,30,h
```

This will send TWO MIT commands back-to-back:
1. First with velocity=2.0 rad/s (current approach)
2. Second with velocity=0 (pure position control)

Share the debug output and note if EITHER variant moves the motor:
- If variant 2 (velocity=0) works but variant 1 (velocity=2.0) doesn't → velocity interpretation issue
- If NEITHER works → protocol/mode issue
- If BOTH work → weird timing or ordering issue

## What to Share

Send me:
1. The enable command debug output with verbose ON
2. Several move command debug outputs (try move,30,0 and move,45,0)
3. The testmove command output
4. What the motor does in each case (nothing? vibrate? try to move then stop?)

This will help identify if it's an encoding issue, protocol issue, or something else.

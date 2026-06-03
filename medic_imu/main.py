import sensor, pyb, math, gc
from lsm6dsox import LSM6DSOX
from machine import Pin, SPI

# ─── USB serial ───────────────────────────────────────────────────────────────
# Disable Ctrl+C interrupt so the IDE can't accidentally stop the script
# while data is streaming. Reconnect via USB and re-flash to update.
usb = pyb.USB_VCP()
usb.setinterrupt(-1)

# ─── Hardware ─────────────────────────────────────────────────────────────────
lsmimu = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
ledpin = Pin("PF4", Pin.OUT_PP, Pin.PULL_UP)

# ─── Binary frame protocol ────────────────────────────────────────────────────
# Frame: MAGIC(4) + TYPE(1) + LENGTH(4 big-endian) + PAYLOAD(N)
# TYPE 0x01 = IMU text line   "IMU,seq,ts_ms,roll,pitch,yaw\n"
# TYPE 0x02 = video JPEG bytes
_MAGIC = b"NICL"
_T_IMU = 0x01
_T_VID = 0x02

def _hdr(ftype, size):
    """9-byte frame header — no heap alloc on the struct bytes."""
    return (_MAGIC
            + bytes([ftype])
            + bytes([(size >> 24) & 0xFF,
                     (size >> 16) & 0xFF,
                     (size >>  8) & 0xFF,
                      size        & 0xFF]))

def send_imu(packet_bytes):
    usb.write(_hdr(_T_IMU, len(packet_bytes)))
    usb.write(packet_bytes)

def send_video(cframe):
    size = cframe.size()
    usb.write(_hdr(_T_VID, size))
    k = 0
    while k < size:
        end = k + 1024
        if end > size:
            end = size
        usb.write(bytes(cframe[k:end]))
        k = end

# ─── Madgwick 6-DOF filter ────────────────────────────────────────────────────
q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
B_madgwick = 0.02

def invSqrt(x):
    return 1.0 / math.sqrt(x)

def Madgwick6DOF(gx, gy, gz, ax, ay, az, dt):
    gx *= 0.0174533; gy *= 0.0174533; gz *= 0.0174533
    global q0, q1, q2, q3
    qDot1 = 0.5 * (-q1*gx - q2*gy - q3*gz)
    qDot2 = 0.5 * ( q0*gx + q2*gz - q3*gy)
    qDot3 = 0.5 * ( q0*gy - q1*gz + q3*gx)
    qDot4 = 0.5 * ( q0*gz + q1*gy - q2*gx)
    if ax * ay * az != 0:
        rN = invSqrt(ax*ax + ay*ay + az*az)
        ax *= rN; ay *= rN; az *= rN
        _2q0=2*q0; _2q1=2*q1; _2q2=2*q2; _2q3=2*q3
        _4q0=4*q0; _4q1=4*q1; _4q2=4*q2
        _8q1=8*q1; _8q2=8*q2
        q0q0=q0*q0; q1q1=q1*q1; q2q2=q2*q2; q3q3=q3*q3
        s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay
        s1 = (_4q1*q3q3 - _2q3*ax + 4*q0q0*q1 - _2q0*ay
              - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az)
        s2 = (4*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay
              - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az)
        s3 = 4*q1q1*q3 - _2q1*ax + 4*q2q2*q3 - _2q2*ay
        rN = invSqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3)
        s0*=rN; s1*=rN; s2*=rN; s3*=rN
        qDot1 -= B_madgwick*s0; qDot2 -= B_madgwick*s1
        qDot3 -= B_madgwick*s2; qDot4 -= B_madgwick*s3
    q0 += qDot1*dt; q1 += qDot2*dt
    q2 += qDot3*dt; q3 += qDot4*dt
    rN = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    q0*=rN; q1*=rN; q2*=rN; q3*=rN
    roll_IMU  = math.atan2(q0*q1+q2*q3, 0.5-q1*q1-q2*q2)  * 57.29577951
    pitch_IMU = -math.asin(-2*(q1*q3-q0*q2))               * 57.29577951
    yaw_IMU   = -math.atan2(q1*q2+q0*q3, 0.5-q2*q2-q3*q3) * 57.29577951
    return (roll_IMU, pitch_IMU, yaw_IMU)

# ─── IMU update ───────────────────────────────────────────────────────────────
imu_seq  = 0
old_roll = 0

def updateIMU(fps):
    global imu_seq, old_roll
    dt = 1.0 / fps
    (rx, ry, rz) = lsmimu.gyro()
    (ax, ay, az) = lsmimu.accel()
    ax *= 9.8; ay *= 9.8; az *= 9.8
    (roll, pitch, yaw) = Madgwick6DOF(rx, ry, rz, ax, ay, az, dt)
    if abs(roll) > 90:
        pitch = 180 - pitch
    if pitch > 50:
        roll = old_roll + rx * dt
    old_roll = roll

    imu_seq += 1
    ts_ms = pyb.millis()
    packet = bytes("IMU,{},{},{:.3f},{:.3f},{:.3f}\n".format(
                   imu_seq, ts_ms, roll, pitch, yaw), "utf-8")
    send_imu(packet)

# ─── Main loop ────────────────────────────────────────────────────────────────
gc.collect()

def start_streaming():
    import time
    clock = time.clock()
    while True:
        clock.tick()
        ledpin.on()

        frame  = sensor.snapshot()
        cframe = frame.compress(quality=35)

        fps = clock.fps()
        try:
            send_video(cframe)
            if fps > 0:
                updateIMU(fps)
        except OSError:
            pass   # USB write failed — host not ready or disconnected

        ledpin.off()

while True:
    try:
        start_streaming()
    except Exception as e:
        print("Error:", e)

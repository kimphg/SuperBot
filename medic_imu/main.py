import sensor
import time
import network
from network import WLAN
import pyb
import socket
from lsm6dsox import LSM6DSOX
from machine import Pin
from machine import SPI
import math

SSID = "AA11"
KEY = "12345678"
HOST = ""
PORT = 8080
HEARTBEAT_TIMEOUT_MS  = 3000   # declare disconnected after 3 s without PING
IMU_BUFFER_DURATION_MS = 5000  # keep at most 5 s of buffered IMU packets

lsmimu = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
ledpin = Pin("PF4", Pin.OUT_PP, Pin.PULL_UP)
wlan = WLAN(network.AP_IF)
roll  = 0
pitch = 90
yaw   = 0
imu_clock = pyb.millis()

def start_ap():
    wlan.active(False)
    time.sleep_ms(500)
    wlan.active(True)
    wlan.config(ssid=SSID, key=KEY, channel=6)
    while not wlan.active():
        print('Starting access point "{:s}"...'.format(SSID))
        ledpin.on()
        time.sleep_ms(1000)
        ledpin.off()
        time.sleep_ms(1000)
    ledpin.off()
    ip, netmask, gateway, dns = wlan.ifconfig()
    print("WiFi status:")
    print("  mode    : AP")
    print("  active  :", wlan.active())
    print("  ssid    :", SSID)
    print("  ip      :", ip)
    print("  netmask :", netmask)
    print("  gateway :", gateway)
    print("  send to : (waiting for first heartbeat)")

def client_count():
    try:
        return len(wlan.status('stations'))
    except (OSError, ValueError, TypeError):
        return -1

start_ap()
time.sleep_ms(5000)
t0 = pyb.millis()
while client_count() == 0:
    if pyb.elapsed_millis(t0) > 30000:
        print("No WiFi client in 30s, restarting AP...")
        start_ap()
        t0 = pyb.millis()
    ledpin.on()
    time.sleep_ms(250)
    ledpin.off()
    time.sleep_ms(250)
print("Client connected, clients:", client_count())

svideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sudp   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Heartbeat socket — Qt sends PING here so firmware knows the host is alive
shb = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
shb.bind(('', 31002))
shb.setblocking(False)  # never block the streaming loop

# Resolved when the first heartbeat arrives (client IP auto-discovered)
addr      = None
addrvideo = None

q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
B_madgwick = 0.02
B_accel    = 0.4
B_gyro     = 0.1

last_heartbeat_ms  = 0
has_ever_connected = False
imu_buffer         = []   # list of (ts_ms: int, packet: bytes)
imu_seq            = 0
old_roll           = 0

def invSqrt(x):
    return 1.0 / math.sqrt(x)

def Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq):
    gx *= 0.0174533
    gy *= 0.0174533
    gz *= 0.0174533
    global q1, q2, q3, q0
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    if ax * ay * az != 0:
        recipNorm = invSqrt(ax * ax + ay * ay + az * az)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0 * q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
        s0 *= recipNorm
        s1 *= recipNorm
        s2 *= recipNorm
        s3 *= recipNorm
        qDot1 -= B_madgwick * s0
        qDot2 -= B_madgwick * s1
        qDot3 -= B_madgwick * s2
        qDot4 -= B_madgwick * s3
    q0 += qDot1 * invSampleFreq
    q1 += qDot2 * invSampleFreq
    q2 += qDot3 * invSampleFreq
    q3 += qDot4 * invSampleFreq
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 *= recipNorm
    q1 *= recipNorm
    q2 *= recipNorm
    q3 *= recipNorm
    roll_IMU  = math.atan2(q0 * q1 + q2 * q3, 0.5 - q1 * q1 - q2 * q2) * 57.29577951
    pitch_IMU = -math.asin(-2.0 * (q1 * q3 - q0 * q2)) * 57.29577951
    yaw_IMU   = -math.atan2(q1 * q2 + q0 * q3, 0.5 - q2 * q2 - q3 * q3) * 57.29577951
    return (roll_IMU, pitch_IMU, yaw_IMU)

def check_heartbeat():
    """Non-blocking — call once per frame to sample the heartbeat socket.
    On first PING, auto-discovers the client IP and resolves send addresses."""
    global last_heartbeat_ms, has_ever_connected, addr, addrvideo
    try:
        data, sender = shb.recvfrom(32)
        if data.startswith(b'PING'):
            last_heartbeat_ms  = pyb.millis()
            if not has_ever_connected:
                client_ip = sender[0]
                addr      = socket.getaddrinfo(client_ip, 31000)[0][4]
                addrvideo = socket.getaddrinfo(client_ip, 31001)[0][4]
                print("Client discovered:", client_ip)
            has_ever_connected = True
    except OSError:
        pass  # EAGAIN — no data ready

def is_connected():
    return has_ever_connected and (pyb.millis() - last_heartbeat_ms < HEARTBEAT_TIMEOUT_MS)

def flush_imu_buffer():
    """Send buffered packets in chronological order.
    On send failure, keep the unsent tail for the next attempt."""
    global imu_buffer
    if addr is None:
        return
    sent = 0
    for _, pkt in imu_buffer:
        try:
            sudp.sendto(pkt, addr)
            sent += 1
        except OSError:
            break
    imu_buffer = imu_buffer[sent:]

def updateIMU(fps):
    global imu_seq, old_roll, imu_buffer
    dtus = 1.0 / fps
    (rx, ry, rz) = lsmimu.gyro()
    (ax, ay, az) = lsmimu.accel()
    ax *= 9.8
    ay *= 9.8
    az *= 9.8
    (roll, pitch, yaw) = Madgwick6DOF(rx, ry, rz, ax, ay, az, dtus)
    if abs(roll) > 90:
        pitch = 180 - pitch
    if pitch > 50:
        roll = old_roll + rx * dtus
    old_roll = roll

    imu_seq += 1
    ts_ms  = pyb.millis()
    # Packet format: IMU,<seq>,<ts_ms>,<roll>,<pitch>,<yaw>
    packet = bytes("IMU,{},{},{:.3f},{:.3f},{:.3f}\n".format(
                   imu_seq, ts_ms, roll, pitch, yaw), 'utf-8')

    if is_connected():
        if imu_buffer:
            flush_imu_buffer()          # replay buffered data first
        try:
            sudp.sendto(packet, addr)
        except OSError:
            imu_buffer.append((ts_ms, packet))
    elif has_ever_connected:
        # WiFi interrupted — buffer and enforce 5-second sliding window
        imu_buffer.append((ts_ms, packet))
        cutoff = ts_ms - IMU_BUFFER_DURATION_MS
        while imu_buffer and imu_buffer[0][0] < cutoff:
            imu_buffer.pop(0)

def start_streaming():
    clock = time.clock()
    while True:
        clock.tick()
        check_heartbeat()

        if is_connected():
            frame  = sensor.snapshot()
            cframe = frame.compress(quality=30)
            try:
                svideo.sendto(b'\x02\x03\x05\x07', addrvideo)
                k = 0
                while k < cframe.size():
                    k1 = k
                    k  = k + 1400
                    if k > cframe.size():
                        k = cframe.size()
                    svideo.sendto(bytes(cframe[k1:k]), addrvideo)
            except OSError:
                pass  # drop the incomplete frame, try again next tick
        else:
            sensor.snapshot()  # keep the sensor pipeline active, discard frame

        updateIMU(clock.fps())

while True:
    try:
        start_streaming()
    except OSError as e:
        print("socket error: ", e)

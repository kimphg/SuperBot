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
SSID = "Star3k"
KEY = "Abach04122019"
CONTROLLER_IP = "192.168.4.16"  # PC's DHCP address on the board's AP (192.168.4.x)
HOST = ""
PORT = 8080
lsmimu = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
ledpin = Pin("PF4", Pin.OUT_PP, Pin.PULL_UP)
wlan = WLAN(network.AP_IF)
wlan.active(True)
wlan.config(ssid=SSID, key=KEY)
roll=0
pitch=90
yaw=0
imu_clock = pyb.millis()
time.sleep_ms(5000)
while not wlan.active():
    print('Starting access point "{:s}"...'.format(SSID))
    ledpin.on()
    time.sleep_ms(1000)
    ledpin.off()
    time.sleep_ms(1000)
ledpin.off()
print("AP started ", wlan.ifconfig())
svideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sudp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
q0 = 1.0
q1 = 0.0
q2 = 0.0
q3 = 0.0
B_madgwick = 0.02
B_accel = 0.4
B_gyro = 0.1
addr = socket.getaddrinfo(CONTROLLER_IP, 31000)[0][4]
addrvideo = socket.getaddrinfo(CONTROLLER_IP, 31001)[0][4]
def invSqrt(x) :
  return 1.0/math.sqrt(x)
def Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq):
    gx *= 0.0174533
    gy *= 0.0174533
    gz *= 0.0174533
    global q1
    global q2
    global q3
    global q0
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    if (ax*ay*az!=0) :
        recipNorm = invSqrt(ax * ax + ay * ay + az * az)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm
        _2q0 = 2.0 * q0
        _2q1 = 2.0 * q1
        _2q2 = 2.0 * q2
        _2q3 = 2.0 * q3
        _4q0 = 4.0 * q0
        _4q1 = 4.0* q1
        _4q2 = 4.0 * q2
        _8q1 = 8.0 * q1
        _8q2 = 8.0 * q2
        q0q0 = q0 * q0
        q1q1 = q1 * q1
        q2q2 = q2 * q2
        q3q3 = q3 * q3
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0* q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0* q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
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
    roll_IMU = math.atan2(q0 * q1 + q2 * q3, 0.5 - q1 * q1 - q2 * q2) * 57.29577951
    pitch_IMU = -math.asin(-2.0 * (q1 * q3 - q0 * q2)) * 57.29577951
    yaw_IMU = -math.atan2(q1 * q2 + q0 * q3, 0.5 - q2 * q2 - q3 * q3) * 57.29577951
    return (roll_IMU,pitch_IMU,yaw_IMU)
old_roll=0
def updateIMU(fps):
    dtus=1.0/fps
    (rx,ry,rz) = lsmimu.gyro()
    (ax,ay,az) = lsmimu.accel()
    ax*=9.8
    ay*=9.8
    az*=9.8
    global old_roll
    (roll,pitch,yaw) = Madgwick6DOF(rx,ry,rz,ax,ay,az,dtus)
    if(abs(roll)>90):
        pitch=180-pitch
    if(pitch>50):
        roll = old_roll+rx*dtus
    old_roll=roll
    print((roll,pitch,yaw))
    sudp.sendto(bytes("IMU,"+str(roll)+","+str(pitch)+","+str(yaw)+"\n", 'utf-8'),addr)
    return True
def start_streaming():
    clock = time.clock()
    header = [0xff,0xff,0xff,0xff]
    while True:
        clock.tick()
        frame = sensor.snapshot()
        cframe = frame.compress(quality=30)
        svideo.sendto(b'\x02\x03\x05\x07',addrvideo)
        k=0
        while (k<cframe.size()):
            k1 = k
            k= k+1400
            if(k>cframe.size()):
                k=cframe.size()
            datagram=bytes(cframe[k1:k])
            svideo.sendto(datagram,addrvideo)
        updateIMU(clock.fps())
while True:
    try:
        start_streaming()
    except OSError as e:
        print("socket error: ", e)

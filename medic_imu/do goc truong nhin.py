# MJPEG Streaming
#
# This example shows off how to do MJPEG streaming to a FIREFOX webrowser
# Chrome, Firefox and MJpegViewer App on Android have been tested.
# Connect to the IP address/port printed out from ifconfig to view the stream.
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
SSID = "AndroidAPs8"  # Network SSID
KEY = "12344321"  # Network key
CONTROLLER_IP = "192.168.248.184"
HOST = ""  # Use first available interface
PORT = 8080  # Arbitrary non-privileged port
lsmimu = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
# Init sensor
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
ledpin = Pin("PF4", Pin.OUT_PP, Pin.PULL_UP)
# Init wlan module and connect to network
wlan = WLAN(network.STA_IF)
wlan.active(True)
#wlan.ifconfig(("192.168.1.18", "255.255.255.0", "192.168.1.1", "192.168.1.1"))
wlan.connect(SSID, KEY)
roll=0
pitch=90
yaw=0
imu_clock = pyb.millis()
time.sleep_ms(5000)
while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(SSID))
    ledpin.on()
    time.sleep_ms(2000)
    wlan.connect(SSID, KEY)
    ledpin.off()
    time.sleep_ms(1000)
ledpin.off()
# We should have a valid IP now via DHCP
print("WiFi Connected ", wlan.ifconfig())

# Create server socket
svideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
sudp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind and listen
#s.bind([HOST, PORT])
#s.listen(5)
q0 = 1.0 #initialize quaternion for madgwick filter
q1 = 0.0
q2 = 0.0
q3 = 0.0

#Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
B_madgwick = 0.02 #Madgwick filter parameter
B_accel = 0.4     #Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
B_gyro = 0.1       #Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
#float B_mag = 1.0;        #Magnetometer LP filter parameter

# Set server socket to blocking
#s.setblocking(True)
addr = socket.getaddrinfo(CONTROLLER_IP, 31000)[0][4]
addrvideo = socket.getaddrinfo(CONTROLLER_IP, 31001)[0][4]
def invSqrt(x) :

  return 1.0/math.sqrt(x)

def Madgwick6DOF(gx, gy, gz, ax, ay, az, invSampleFreq):
#    #Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533
    gy *= 0.0174533
    gz *= 0.0174533

    global q1
    global q2
    global q3
    global q0
    #Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)
    if (ax*ay*az!=0) :

    #        #Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm

    #        #Auxiliary variables to avoid repeated arithmetic
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

    #        #Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
        s2 = 4.0* q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
        s3 = 4.0* q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)# #normalise step magnitude
        s0 *= recipNorm
        s1 *= recipNorm
        s2 *= recipNorm
        s3 *= recipNorm

    #        #Apply feedback step
        qDot1 -= B_madgwick * s0
        qDot2 -= B_madgwick * s1
        qDot3 -= B_madgwick * s2
        qDot4 -= B_madgwick * s3


#    #Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * invSampleFreq
    q1 += qDot2 * invSampleFreq
    q2 += qDot3 * invSampleFreq
    q3 += qDot4 * invSampleFreq

#    #Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    q0 *= recipNorm
    q1 *= recipNorm
    q2 *= recipNorm
    q3 *= recipNorm

#    #compute angles - NWU
    roll_IMU = math.atan2(q0 * q1 + q2 * q3, 0.5 - q1 * q1 - q2 * q2) * 57.29577951 #degrees
    pitch_IMU = -math.asin(-2.0 * (q1 * q3 - q0 * q2)) * 57.29577951                #degrees
    yaw_IMU = -math.atan2(q1 * q2 + q0 * q3, 0.5 - q2 * q2 - q3 * q3) * 57.29577951 #degrees
    return (roll_IMU,pitch_IMU,yaw_IMU)
old_roll=0
def updateIMU(fps):
    dtus=1.0/fps
    (rx,ry,rz) = lsmimu.gyro()
    (ax,ay,az) = lsmimu.accel()
#    lsmimu.euler_angles()
    ax*=9.8
    ay*=9.8
    az*=9.8
    global old_roll
#    lsmimu.read_e
#    print((ax,ay,az))
    (roll,pitch,yaw) = Madgwick6DOF(rx,ry,rz,ax,ay,az,dtus)
    if(abs(roll)>90):
        pitch=180-pitch

    if(pitch>50):
        roll = old_roll+rx*dtus
    old_roll=roll
    print((roll,pitch,yaw))
#    print((int(rx),int(ry),int(rz)))
    sudp.sendto(bytes("IMU,"+str(roll)+","+str(pitch)+","+str(yaw)+"\n", 'utf-8'),addr)
    return True

def start_streaming():


    # FPS clock
    clock = time.clock()

    header = [0xff,0xff,0xff,0xff]
    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.
    while True:
        clock.tick()  # Track elapsed milliseconds between snapshots().
        frame = sensor.snapshot()

        cframe = frame.compressed(quality=30)

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

#time.sleep_ms(1000)

#def timer_callback(timer):                # we will receive the timer object when being called
#    lsmimu.read_gyro()

#tim = pyb.Timer(4, freq=10)      # create a timer object using timer 4 - trigger at 100Hz
#tim.callback(timer_callback)
#time.sleep_ms(1000)

while True:
    try:
        start_streaming()
    except OSError as e:
        print("socket error: ", e)
        # sys.print_exception(e)

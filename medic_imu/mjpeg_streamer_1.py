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
SSID = "Phuong Nga"  # Network SSID
KEY = "19891990"  # Network key
HOST = ""  # Use first available interface
PORT = 8080  # Arbitrary non-privileged port
lsmimu = LSM6DSOX(SPI(5), cs_pin=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))
# Init sensor
sensor.reset()
sensor.set_framesize(sensor.VGA)
sensor.set_pixformat(sensor.RGB565)

# Init wlan module and connect to network
wlan = WLAN(network.STA_IF)
wlan.active(True)
wlan.ifconfig(("192.168.1.18", "255.255.255.0", "192.168.1.1", "192.168.1.1"))
wlan.connect(SSID, KEY)
#wlan.config(ssid=SSID)
while not wlan.isconnected():
    print('Trying to connect to "{:s}"...'.format(SSID))
    time.sleep_ms(2000)
    wlan.connect(SSID, KEY)
    time.sleep_ms(1000)

# We should have a valid IP now via DHCP
print("WiFi Connected ", wlan.ifconfig())

# Create server socket
svideo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, True)
sudp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# Bind and listen
#s.bind([HOST, PORT])
#s.listen(5)

# Set server socket to blocking
#s.setblocking(True)
roll=0
pitch=0
yaw=0

def start_streaming():


    # FPS clock
    clock = time.clock()
    addr = socket.getaddrinfo("192.168.1.14", 31000)[0][4]
    addrvideo = socket.getaddrinfo("192.168.1.14", 31001)[0][4]
    header = [0xff,0xff,0xff,0xff]
    # Start streaming images
    # NOTE: Disable IDE preview to increase streaming FPS.
    while True:
        clock.tick()  # Track elapsed milliseconds between snapshots().
        frame = sensor.snapshot()

        cframe = frame.compressed(quality=50)

        svideo.sendto(b'\x02\x03\x05\x07',addrvideo)
        k=0
        while (k<cframe.size()):
            k1 = k
            k= k+1400
            if(k>cframe.size()):
                k=cframe.size()
            datagram=bytes(cframe[k1:k])
            svideo.sendto(datagram,addrvideo)

        (rx,ry,rz) = lsmimu.read_gyro()
        (ax,ay,az) = lsmimu.read_accel()
        sudp.sendto(bytes("IMU,"+str(ax)+","+str(ay)+","+str(az)+","+str(rx)+","+str(ry)+","+str(rz)+"\n", 'utf-8'),addr)
#        sudp.sendto(bytes("IMU,"+str(lsm.read_roll()),'utf-8'))

        print(clock.fps())

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

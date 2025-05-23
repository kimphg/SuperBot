# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

import sensor, image, time
from pyb import Pin, Timer
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
def setMode(mode):
    if(mode==1):
        sensor.set_framesize(sensor.QVGA) # we run out of memory if the resolution is much bigger...
        sensor.skip_frames(time = 500)
        sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
        sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
        sensor.set_auto_exposure(     False, exposure_us=4000 )
        sensor.set_auto_gain(False, gain_db=10)
        sensor.skip_frames(time = 500)
    if(mode==2):
        sensor.set_framesize(sensor.QVGA) # we run out of memory if the resolution is much bigger...
        sensor.skip_frames(time = 500)
        sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
        sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
        sensor.set_auto_exposure(False, exposure_us=4000 )
        sensor.set_auto_gain(False, gain_db=28)
        sensor.skip_frames(time = 500)
    if(mode==3):
        sensor.set_framesize(sensor.VGA) # we run out of memory if the resolution is much bigger...
        sensor.skip_frames(time = 500)
        sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
        sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
        sensor.set_auto_exposure(False, exposure_us=10000 )
        sensor.set_auto_gain(False, gain_db=20)
        sensor.skip_frames(time = 500)
setMode(1)
clock = time.clock()
from pyb import UART

uart = UART(3, 921600, timeout_char=1000)                         # init with given baudrate
uart.init(921600, bits=8, parity=None, stop=1, timeout_char=1000) # init with given parameters
# Note! Unlike find_qrcodes the find_apriltags method does not need lens correction on the image to work.

# The apriltag code supports up to 6 tag families which can be processed at the same time.
# Returned tag objects will have their tag family and id within the tag family.
def crc16(data : bytearray, offset , length):
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0,8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc & 0xFFFF
def crc8(data : bytearray, offset , length):
    cs=0
    for i in range(0, length):
        cs ^= data[offset + i]
    return cs
tag_families = 0
#tag_families |= image.TAG16H5 # comment out to disable this family
#tag_families |= image.TAG25H7 # comment out to disable this family
#tag_families |= image.TAG25H9 # comment out to disable this family
#tag_families |= image.TAG36H10 # comment out to disable this family
tag_families |= image.TAG36H11 # comment out to disable this family (default family)
#tag_families |= image.ARTOOLKIT # comment out to disable this family

# What's the difference between tag families? Well, for example, the TAG16H5 family is effectively
# a 4x4 square tag. So, this means it can be seen at a longer distance than a TAG36H11 tag which
# is a 6x6 square tag. However, the lower H value (H5 versus H11) means that the false positve
# rate for the 4x4 tag is much, much, much, higher than the 6x6 tag. So, unless you have a
# reason to use the other tags families just use TAG36H11 which is the default family.
sensor.set_vflip(True)
def family_name(tag):
    if(tag.family() == image.TAG16H5):
        return "TAG16H5"
    if(tag.family() == image.TAG25H7):
        return "TAG25H7"
    if(tag.family() == image.TAG25H9):
        return "TAG25H9"
    if(tag.family() == image.TAG36H10):
        return "TAG36H10"
    if(tag.family() == image.TAG36H11):
        return "TAG36H11"
    if(tag.family() == image.ARTOOLKIT):
        return "ARTOOLKIT"
p = Pin('P8') # P4 has TIM2, CH3
tim = Timer(4, freq=1000)
ch = tim.channel(2, Timer.PWM, pin=p)
ch.pulse_width_percent(50)
p1 = Pin('P9') # P4 has TIM2, CH3
p1.init(Pin.IN,pull = Pin.PULL_UP)
count=0
uartBuff = ""
newFrameDetected = False
newFramePos = 0
masterFrameLen=6
workMode=1
frameWidth = sensor.width()
frameHeight = sensor.height()
fps_cam=0
uart.init(921600)
last_tag_id = -1
stable_count = 0
while(True):
    clock.tick()
#    print(sensor.get_exposure_us())
    img = sensor.snapshot()
    img.lens_corr(strength =1.5, zoom = 1)
    #img.save ("example.jpg")

    if(workMode>=1):#high speed mode
        tagCount=0
        packet="$CAM1,"
        packet+=(str(workMode))
        packet+=(",")
        packet+=(str(fps_cam))
        packet+=(",")
        for tag in img.find_apriltags(families=tag_families):
            packet += ("TD")
            packet += (",")

            packet += (str(tag.id))
            packet += (",")
            packet += (str(int(tag.cx/frameWidth*100)))
            packet += (",")
            packet += (str(int(tag.cy/frameHeight*100)))
            packet += (",")
            packet += (str(int(tag.rotation*1800.0/3.141592653589793)))
            packet += (",")
#            if(last_tag_id == tag.id()):
#                stable_count=stable_count+1
#            else:
#                last_tag_id = tag.id()
#                stable_count=0
#            packet += (str(stable_count))
#            packet += (",")
        datalen = len(packet);
        packetBytes = bytearray(packet,'ascii')
        cs_byte = crc8(packetBytes,0,datalen)
        packet += (str(cs_byte))
        packet += ("#")
        packet += ("\n")
        tagCount=tagCount+1
        uart.write(packet)
        print(packet)
        # defaults to TAG36H11 without "families".
#        img.draw_rectangle(tag.rect(), color = (255))
#        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
#        print_args = ("CAMB","DET", tag.id(), (180 * tag.rotation()) / math.pi)
#        print("%s,%s,%d,%f#" % print_args)
#        count=count+1
#    Serial
#    datalen = uart.any()
#    if(datalen):
#        if(len(uartBuff)+datalen>1000):
#            uartBuff= uart.read(datalen)
#            continue
#        inputbyte = uart.read(datalen)
##        print(inputbyte)
##        if(inputbyte>=33):
##        uartBuff+=(inputbyte.decode(encoding="utf-8", errors="ignore"))
##        print(uartBuff)
#    if(len(uartBuff)>1000):
#        uartBuff=""
    fps_cam = int(clock.fps())

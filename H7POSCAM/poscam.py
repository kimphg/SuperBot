# AprilTags Example
#
# This example shows the power of the OpenMV Cam to detect April Tags
# on the OpenMV Cam M7. The M4 versions cannot detect April Tags.

import sensor, image, time, math
from pyb import Pin, Timer
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA) # we run out of memory if the resolution is much bigger...
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)  # must turn this off to prevent image washout...
sensor.set_auto_whitebal(False)  # must turn this off to prevent image washout...
clock = time.clock()
from pyb import UART

uart = UART(1, 1000000, timeout_char=1000)                         # init with given baudrate
uart.init(1000000, bits=8, parity=None, stop=1, timeout_char=1000) # init with given parameters
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
uartBuff = []
newFrameDetected = False
newFramePos = 0
masterFrameLen=6
workMode=0
frameWidth = sensor.width()
frameHeight = sensor.height()
while(True):
    clock.tick()

    img = sensor.snapshot()

    #img.save ("example.jpg")
    if(workMode>0):
        tagCount=0
        for tag in img.find_apriltags(families=tag_families):
            packet=[]
            packet.append(0xAA)
            packet.append(0x55)
            packet.append(0x01)
            packet.append(0x11)
            packet.append(byte())
            packet.append(byte(tag.id()>>8))
            packet.append(byte(tag.id()&0xff))
            packet.append(byte(tag.cx()/frameWidth*255))
            packet.append(byte(tag.cy()/frameHeight*255))
            rotationDeg = (180 * tag.rotation()) / math.pi*10
            if(rotationDeg<0)rotationDeg+=3600
            packet.append(byte(rotationDeg>>8))
            packet.append(byte(rotationDeg&0xff))
            datalen = packet.size()-2;
            cs_byte = crc16(packet,2,datalen)
            packet.append(cs_byte>>8)
            packet.append(cs_byte&0xff)
            tagCount++
            uart.write(packet)
        # defaults to TAG36H11 without "families".
#        img.draw_rectangle(tag.rect(), color = (255))
#        img.draw_cross(tag.cx(), tag.cy(), color = (0, 255, 0))
#        print_args = ("CAMB","DET", tag.id(), (180 * tag.rotation()) / math.pi)
#        print("%s,%s,%d,%f#" % print_args)
#        count=count+1
#    Serial
    datalen = uart.any()
    if(datalen):
        uartBuff.append(uart.read(datalen))
        for i in range(uartBuff.size()-1):
            if(uartBuff[i]==0xAA):
                if(uartBuff[i+1]==0x55):
                    newFrameDetected=True
                    newFramePos=i
    if(newFrameDetected):
        if(newFramePos<=(uartBuff.size()-6)):
            addressByte = uartBuff[newFramePos+2]
            if(addressByte==0x11):
                commandByte=uartBuff[newFramePos+3]
                if(commandByte=0x01):
                    workMode=1
                    sensor.set_pixformat(sensor.GRAYSCALE)
                    sensor.set_framesize(sensor.QVGA)
                else if(commandByte=0x02):
                    workMode=2
                    sensor.set_pixformat(sensor.GRAYSCALE)
                    sensor.set_framesize(sensor.QVGA)
                else:
                    workMode=0
    print(clock.fps())

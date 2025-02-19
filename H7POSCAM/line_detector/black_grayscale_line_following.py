import sensor
import time
import math
ROIS = [
    (0, 100, 160, 20, 0.7),
    (0, 50, 160, 20, 0.3),
    (0, 0, 160, 20, 0.1),
]
ROIS2 = [
    ( 140,0 ,20, 120,  0.7),
    ( 70 ,0 ,20, 120,  0.3),
    (0   , 0,20, 120,  0.1),
]
weight_sum = 0
for r in ROIS:
    weight_sum += r[4]
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(False)
clock = time.clock()
from pyb import UART
import pyb
red_led = pyb.LED(1)
green_led = pyb.LED(2)
uart = UART(1, 112500, timeout_char=1000)
uart.init(112500, bits=8, parity=None, stop=1, timeout_char=1000)
while True:
    clock.tick()
    img = sensor.snapshot()
    img.lens_corr(1.8)
    centroid_sum = 0
    h = img.histogram().get_statistics()
    thresh = h.median()+h.stdev()
    if(thresh>255):
        thresh=255
    maxblobV=[]
    for r in ROIS:
        blobs = img.find_blobs(
            [(thresh, 255)], roi=r[0:4], merge=True
        )
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            if(largest_blob.pixels()>400):
                maxblobV.append(largest_blob)
    maxblobH=[]
    for r in ROIS2:
        blobs = img.find_blobs(
            [(thresh, 255)], roi=r[0:4], merge=True
        )
        if blobs:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            if(largest_blob.pixels()>400):
                maxblobH.append(largest_blob)
    red_led.off()
    green_led.off()
    if(len(maxblobV)==3):
        red_led.on()

        widest_blob = max(maxblobV, key=lambda b: b.w())
        img.draw_cross(maxblobV[0].cx(), maxblobV[0].cy())
        img.draw_cross(maxblobV[1].cx(), maxblobV[1].cy())
        img.draw_cross(maxblobV[2].cx(), maxblobV[2].cy())
        dx = maxblobV[0].cx()-maxblobV[2].cx()
        dy = maxblobV[2].cy()-maxblobV[0].cy()
        ctx = (maxblobV[0].cx()+maxblobV[2].cx())/320.0
        angle1 = math.degrees(math.atan(dx/dy))
        uart.write('$VL,')
        uart.write("%f," % ctx)
        uart.write(',')
        uart.write("%f," %angle1)
        uart.write(',#\n')
    if(len(maxblobH)==3):
        green_led.on()
        img.draw_cross(maxblobH[0].cx(), maxblobH[0].cy())
        img.draw_cross(maxblobH[1].cx(), maxblobH[1].cy())
        img.draw_cross(maxblobH[2].cx(), maxblobH[2].cy())
        dx = maxblobH[0].cx()-maxblobH[2].cx()
        dy = maxblobH[2].cy()-maxblobH[0].cy()
        if(abs(dy)<1):
            angle1=90
        else:
            angle1 = math.degrees(math.atan(dx/dy))
        ctx = (maxblobH[0].cy()+maxblobH[2].cy())/240.0
        uart.write('$HL,')
        uart.write("%f," % ctx)
        uart.write(',')
        uart.write("%f," %angle1)
        uart.write(',#\n')
    print(clock.fps())

# Snapshot Example
#
# Note: You will need an SD card to run this example.
#
# You can use your OpenMV Cam to save image files.

import sensor, image, pyb,time,mjpeg
from pyb import Pin, Timer

RED_LED_PIN = 1
BLUE_LED_PIN = 3
pyb.LED(RED_LED_PIN).on()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HD) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 1000) # Let new settings take affect.
m = mjpeg.Mjpeg("abs.mjpeg")
pyb.LED(RED_LED_PIN).on()
uart = pyb.UART(3, 500000)
clock = time.clock() # Tracks FPS.
pyb.LED(RED_LED_PIN).off()
framid = ""
count=0

while (True):
    inputint = uart.readchar()
    if(inputint>=0):
        bytein = chr(inputint)

        if(bytein=="$"):
            framid = ""
        elif (bytein=="#"):
            #clock.tick()
            #print(framid)
            m.add_frame(sensor.snapshot())
            #count=count+1
            #print(count)
        elif (bytein=="!"):
            break
        elif (bytein=="@"):
            m = mjpeg.Mjpeg(framid+"sl.mjpeg")
            pyb.LED(BLUE_LED_PIN).on()
        else:
            framid = framid+bytein
m.close(20)
pyb.LED(BLUE_LED_PIN).off()

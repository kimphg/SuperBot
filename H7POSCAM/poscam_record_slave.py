# Snapshot Example
#
# Note: You will need an SD card to run this example.
#
# You can use your OpenMV Cam to save image files.

import sensor, image, pyb,time
from pyb import Pin, Timer

RED_LED_PIN = 1
BLUE_LED_PIN = 3
pyb.LED(RED_LED_PIN).on()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HD) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 1000) # Let new settings take affect.
m = mjpeg.Mjpeg(str(pyb.rng())+".mjpeg")
pyb.LED(RED_LED_PIN).on()
uart = pyb.UART(3, 500000)

pyb.LED(RED_LED_PIN).off()
pyb.LED(BLUE_LED_PIN).on()


while (True):
    inputint = uart.readchar()
    if(inputint>=0):
        bytein = chr()
        if(bytein=="$"):
            framid = ""
        elif (bytein=="#"):
            m.add_frame(sensor.snapshot(),quality = 50)
        elif (bytein=="!"):
            break
        else:
            framid = framid+bytein
        #pyb.LED(BLUE_LED_PIN).off()
m.close(14)
pyb.LED(BLUE_LED_PIN).off()

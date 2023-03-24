# Snapshot Example
#
# Note: You will need an SD card to run this example.
#
# You can use your OpenMV Cam to save image files.

import sensor, image, pyb,time
from pyb import Pin, Timer
p2  = pyb.Pin("P2", pyb.Pin.IN)
p3  = pyb.Pin("P3", pyb.Pin.IN)
RED_LED_PIN = 1
BLUE_LED_PIN = 3
clock = time.clock()
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.VGA) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.

pyb.LED(RED_LED_PIN).on()
sensor.skip_frames(time = 2000) # Give the user time to get ready.

pyb.LED(RED_LED_PIN).off()
#pyb.LED(BLUE_LED_PIN).on()

print("You're on camera!")
count = 0
p2old = 0
while (True):
    if(p2.value()>p2old):
        sensor.snapshot().save(str(count)+"slave.jpg",quality=50) # or "example.bmp" (or others)
        count=count+1
    p2old = p2.value()
    if(p3.value()>0):
        count=0
print("Done! Reset the camera to see the saved image.")

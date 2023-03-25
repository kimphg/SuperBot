# Snapshot Example
#
# Note: You will need an SD card to run this example.
#
# You can use your OpenMV Cam to save image files.

import sensor, image, pyb,time
from pyb import Pin, Timer
p2  = pyb.Pin("P2", pyb.Pin.OUT_PP)
p3  = pyb.Pin("P3", pyb.Pin.IN)
p3.high()
RED_LED_PIN = 1
BLUE_LED_PIN = 3
clock = time.clock()
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.VGA) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.

pyb.LED(RED_LED_PIN).on()
sensor.skip_frames(time = 2000) # Give the user time to get ready.
uart = pyb.UART(3, 1000000)
#pyb.LED(RED_LED_PIN).off()
pyb.LED(BLUE_LED_PIN).off()

print("You're on camera!")
count = 0
pyb.delay(100)
p3.low()
while(p3.value()==0):
    continue

while (count<200):

    uart.write('$'+str(count)+'#')
    #p2.high()
    img = sensor.snapshot()
    #p2.low()
    img.save(str(count)+"msrec.jpg",quality=50)
    count=count+1

pyb.LED(BLUE_LED_PIN).off()
print("Done! Reset the camera to see the saved image.")

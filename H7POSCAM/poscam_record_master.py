
import sensor, image, pyb,time,mjpeg
from pyb import Pin, Timer
RED_LED_PIN = 1
BLUE_LED_PIN = 3
clock = time.clock()
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.RGB565) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.VGA) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.

pyb.LED(RED_LED_PIN).on()
uart = pyb.UART(3, 1000000)
pyb.LED(BLUE_LED_PIN).off()
m = mjpeg.Mjpeg("example0.mjpeg")
rand_file_num = pyb.rng()
count = 0
pyb.delay(100)

while (count<200):
    clock.tim
    #uart.write('$'+str(count)+'#')
    m.add_frame(sensor.snapshot(),quality = 60)
    print(clock.fps())
    #count=count+1
m.close(20)
pyb.LED(BLUE_LED_PIN).off()
print("Done! Reset the camera to see the saved image.")

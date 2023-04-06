# MJPEG Video Recording Example
#
# Note: You will need an SD card to run this demo.
#
# You can use your OpenMV Cam to record mjpeg files. You can either feed the
# recorder object JPEG frames or RGB565/Grayscale frames. Once you've finished
# recording a Mjpeg file you can use VLC to play it. If you are on Ubuntu then
# the built-in video player will work too.

import sensor, image, time, mjpeg, pyb

RED_LED_PIN = 1
BLUE_LED_PIN = 3
pyb.LED(RED_LED_PIN).on()
pyb.LED(BLUE_LED_PIN).off()
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # or sensor.GRAYSCALE
sensor.set_framesize(sensor.HD) # or sensor.QQVGA (or others)
sensor.skip_frames(time = 2000) # Let new settings take affect.
clock = time.clock() # Tracks FPS.
uart = pyb.UART(3, 500000)
pyb.LED(RED_LED_PIN).on()
sensor.skip_frames(time = 2000) # Give the user time to get ready.

pyb.LED(RED_LED_PIN).off()
pyb.LED(BLUE_LED_PIN).on()
filename = str(pyb.rng())
m = mjpeg.Mjpeg(filename+"ms.mjpeg")
uart.write('$'+filename+'@')
count = 0
fps = 20
mspf = round(1000/fps)
sensor.skip_frames(time = 100)
for i in range(4200):
    frame_start = pyb.millis()
    clock.tick()
    uart.write('$'+str(count)+'#')
    m.add_frame(sensor.snapshot())
    frame_end  = pyb.millis()
    delta_time = frame_end - frame_start
    count=count+1
    if (delta_time < mspf):
        pyb.delay(mspf - delta_time-1)
    print(clock.fps())
uart.write('!')
m.close(fps)
pyb.LED(BLUE_LED_PIN).off()
print("Done! Reset the camera to see the saved recording.")

import sensor, image, time, pyb,mjpeg
RED_LED_PIN = 1
BLUE_LED_PIN = 3
pyb.LED(RED_LED_PIN).on()
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.HD)   # Set frame size to QVGA (320x240)
#sensor.set_windowing((0, 0, 160, 120))
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.
uart = pyb.UART(3, 115200)
pyb.LED(RED_LED_PIN).off()
#img_writer = image.ImageIO(str(pyb.rng())+'.img','w')
m = mjpeg.Mjpeg(str(pyb.rng())+".mjpeg")
i = 0
count = 0
fps = 14
mspf = round(1000/fps)
pyb.LED(BLUE_LED_PIN).on()
while (count<100):
    frame_start = pyb.millis()
    uart.write('$'+str(count)+'#')
    #clock.tick()
    img = sensor.snapshot()
    m.add_frame(sensor.snapshot(),quality = 50)
    frame_end  = pyb.millis()
    delta_time = frame_end - frame_start

    count=count+1
    if (delta_time < mspf):
        pyb.delay(mspf - delta_time-1)
    #print(clock.fps())
m.close(14)
pyb.LED(BLUE_LED_PIN).off()
#img_writer.close()

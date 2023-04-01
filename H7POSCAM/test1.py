import sensor, image, time, pyb,mjpeg

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.VGA)   # Set frame size to QVGA (320x240)
#sensor.set_windowing((0, 0, 160, 120))
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
clock = time.clock()                # Create a clock object to track the FPS.


#img_writer = image.ImageIO(str(pyb.rng())+'.img','w')
m = mjpeg.Mjpeg(str(pyb.rng())+".mjpeg")
i = 0

fps = 20
mspf = round(1000/fps)

while (i < 250):
    #frame_start = pyb.millis()
    clock.tick()
    img = sensor.snapshot()
    m.add_frame(sensor.snapshot(),quality = 60)
    frame_end = pyb.millis()
    #delta_time = frame_end - frame_start

    i = i + 1
    #if (delta_time < mspf):
        #pyb.delay(mspf - delta_time-1)
    print(clock.fps())
m.close(20)
#img_writer.close()

# Black Grayscale Line Following Example
#
# Making a line following robot requires a lot of effort. This example script
# shows how to do the machine vision part of the line following robot. You
# can use the output from this script to drive a differential drive robot to
# follow a line. This script just generates a single turn value that tells
# your robot to go left or right.
#
# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.

import sensor
import time
import math

# Tracks a black line. Use [(128, 255)] for a tracking a white line.
#GRAYSCALE_THRESHOLD =

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [  # [ROI, weight]
    (0, 100, 160, 20, 0.7),  # You'll need to tweak the weights for your app
    (0, 50, 160, 20, 0.3),  # depending on how your robot is setup.
    (0, 0, 160, 20, 0.1),
]
ROIS2 = [  # [ROI, weight]
    ( 140,0 ,20, 120,  0.7),  # You'll need to tweak the weights for your app
    ( 70 ,0 ,20, 120,  0.3),  # depending on how your robot is setup.
    (0   , 0,20, 120,  0.1),
]
# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0
for r in ROIS:
    weight_sum += r[4]  # r[4] is the roi weight.

# Camera setup...
sensor.reset()  # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE)  # use grayscale.
sensor.set_framesize(sensor.QQVGA)  # use QQVGA for speed.
sensor.skip_frames(time=2000)  # Let new settings take affect.
sensor.set_auto_exposure(True)
sensor.set_auto_gain(True)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()  # Tracks FPS.
from pyb import UART

uart = UART(3, 112500, timeout_char=1000)                         # init with given baudrate
uart.init(112500, bits=8, parity=None, stop=1, timeout_char=1000) # init with given parameters
while True:
    clock.tick()  # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot()  # Take a picture and return the image.
    img.lens_corr(1.8)
    centroid_sum = 0
    h = img.histogram().get_statistics()

    thresh = h.median()+h.stdev()
#    print((thresh))
    if(thresh>255):
        thresh=255
    maxblobV=[]
    for r in ROIS:
        blobs = img.find_blobs(
            [(thresh, 255)], roi=r[0:4], merge=True
        )  # r[0:4] is roi tuple.


        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())
            maxblobV.append(largest_blob)
            # Draw a rect around the blob.
    maxblobH=[]
    for r in ROIS2:
        blobs = img.find_blobs(
            [(thresh, 255)], roi=r[0:4], merge=True
        )  # r[0:4] is roi tuple.


        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())
            maxblobH.append(largest_blob)

#            centroid_sum += largest_blob.cx() * r[4]  # r[4] is the roi weight.
    if(len(maxblobV)==3):
#        img.draw_rectangle(maxblobs[0].rect())
#        img.draw_rectangle(maxblobs[2].rect())
        widest_blob = max(maxblobV, key=lambda b: b.w())
#        img.draw_rectangle(widest_blob.)
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
#        img.draw_rectangle(maxblobs[0].rect())
#        img.draw_rectangle(maxblobs[2].rect())
#        widest_blob = max(maxblobs, key=lambda b: b.w())
#        img.draw_rectangle(widest_blob.)
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
#        print(ctx)
        uart.write('$HL,')
        uart.write("%f," % ctx)
        uart.write(',')
        uart.write("%f," %angle1)
        uart.write(',#\n')
#    center_pos = centroid_sum / weight_sum  # Determine center of line.
#    print(center_pos)
    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
#    deflection_angle = 0

    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
#    deflection_angle = -math.atan((center_pos - 80) / 60)

    # Convert angle in radians to degrees.
#    deflection_angle = math.degrees(deflection_angle)

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
#    print("Turn Angle: %f" % deflection_angle)

    print(clock.fps())  # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.

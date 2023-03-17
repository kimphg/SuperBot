# Untitled - By: Phuong - Fri Mar 17 2023

import sensor, image, time
from pyb import Pin, Timer
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

import pyb

red_led = pyb.LED(1)
red_led.on()
green_led = pyb.LED(2)
green_led.on()
blue_led = pyb.LED(3)
blue_led.on()
ir_leds = pyb.LED(4)
#ir_leds.on()

p = Pin('P8') # P4 has TIM2, CH3
tim = Timer(4, freq=1000)
ch = tim.channel(2, Timer.PWM, pin=p)
ch.pulse_width_percent(50)
p1 = Pin('P9') # P4 has TIM2, CH3
p1.init(Pin.IN,pull = Pin.PULL_UP)
frames=0
while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())
    frames=frames+3
    if(frames>=50):
        frames=0
    ch.pulse_width_percent(int(abs(frames-25))+1)

#!/usr/bin/env python3
# pyright: reportMissingImports=false
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import time

# turn off all status LEDs
for i in [11, 25, 24]:
    led = DigitalInOut(eval('board.D'+str(i)))
    led.direction = Direction.OUTPUT
    led.value = True

# blink LED 1
while True:
    led.value = True
    time.sleep(1)
    led.value = False
    time.sleep(1)

    
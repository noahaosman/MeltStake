#!/usr/bin/env python3
# pyright: reportMissingImports=false
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import time

# init LED GPIO
led = DigitalInOut(board.D11)
led.direction = Direction.OUTPUT
led.value = True

# init Leak sensor GPIO
leak = DigitalInOut(board.D27)
leak.direction = Direction.INPUT
leak.pull = Pull.DOWN

with open('/home/pi/MeltStake/LeakState.txt', "w") as f:  
    # write new data line
    f.write("False")

while leak.value == False:
    time.sleep(0.25)

with open('/home/pi/MeltStake/LeakState.txt', "w") as f:  
    # write new data line
    f.write("True")

# blink LED 3
while True:
    led.value = False
    time.sleep(0.1)
    led.value = True
    time.sleep(0.1)
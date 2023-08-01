#!/usr/bin/env python3
# pyright: reportMissingImports=false
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import time
import os

PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT

# watch for meltstake service to start, when it does arm the pwm driver
while True:
    time.sleep(0.25)
    status = os.system('systemctl is-active --quiet meltstake')
    if status == 0:
        PWM_OE.value = False  # armed
    else:
        PWM_OE.value = True  # disarmed


    
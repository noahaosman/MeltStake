# pyright: reportMissingImports=false
import board
from digitalio import DigitalInOut, Direction

PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT
PWM_OE.value = True  # disarmed

print("PWM driver disarmed")
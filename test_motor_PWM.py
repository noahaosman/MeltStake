# pyright: reportMissingImports=false
import busio
import board
from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_pca9685 import PCA9685  # PCA9685 module (PWM driver)
from adafruit_ads1x15.analog_in import AnalogIn
from digitalio import DigitalInOut, Direction, Pull  # GPIO module

i2c_bus4 = I2C(4)
PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT
PWM_OE.value = False  # armed

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c_bus4)
pca.reference_clock_speed = 25000000  # Set the PWM frequency (Default 25000000)
pca.frequency = 200  # Set the PWM duty cycle.

while True:
    usr_in_mot = input("Motor number (0 or 1) :: ")
    usr_in_PWM = input("PWM inp (1100 <--> 1900; 1500 for OFF) :: ")
#while True:
    usr_in_CLK = input("CLK SPD (DEF 2.5) :: ")
    pca.reference_clock_speed = int(float(usr_in_CLK)*10000000)
    print(int(float(usr_in_CLK)*10000000))
    try:
        pwm_input = float(usr_in_PWM)
        print(pwm_input)
        pca.channels[int(usr_in_mot)].duty_cycle = \
            int(pca.frequency*(10**-6)*pwm_input*65535)
    except Exception as e:
        print(e)
        print("bad input, try again")

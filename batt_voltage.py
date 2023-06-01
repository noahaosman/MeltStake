# pyright: reportMissingImports=false
import busio
import board
from adafruit_extended_bus import ExtendedI2C as I2C
import time
import argparse
from datetime import datetime, timezone
import numpy as np
from threading import Thread
from adafruit_pca9685 import PCA9685  # PCA9685 module (PWM driver)
import adafruit_ads1x15.ads1115 as ADS  # ADS1115 module (ADC)
from adafruit_ads1x15.analog_in import AnalogIn
from digitalio import DigitalInOut, Direction, Pull  # GPIO module


i2c_bus4 = I2C(4)
PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT
PWM_OE.value = False  # armed

# parse arguements
argParser = argparse.ArgumentParser()
argParser.add_argument("-d", "--device", help="Melt Stake number")
args = argParser.parse_args()


if args.device == '02':
	ads_bus = busio.I2C(board.SCL, board.SDA)
	ads_addr = 0x49
	BATT_VOLT_DIV_RATIO = (1 + 5.1) / 1  # R1 = 5.1kOhm; R2 = 1kOhm
elif args.device == '03':
	ads_bus = I2C(6)
	ads_addr = 0x48
	self.BATT_VOLT_DIV_RATIO = (3.3 + 10) / 3.3  # R1 = 10kOhm; R2 = 3.3kOhm
else:
	print("ERROR -- INVALID DEVICE!!")

# Create the I2C bus interface.
ads = ADS.ADS1115(ads_bus, address=ads_addr)

batt_voltage = AnalogIn(ads, ADS.P3)

voltage = batt_voltage.voltage * BATT_VOLT_DIV_RATIO


print("Battery Voltage : " + f"{voltage:.2f}" + "V")

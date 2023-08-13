# pyright: reportMissingImports=false
import busio
import os
import re
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

def capP(V):
    p1 = -19.5947850880173
    p2 = 1556.06233071877
    p3 = -49394.7555340183
    p4 = 783435.213797136
    p5 = -6208434.50531532
    p6 = 19665053.4764426
    out = p1*V**5 + p2*V**4 + p3*V**3 + p4*V**2 + p5*V + p6
    return out

i2c_bus4 = I2C(4)

Allbuses = [f for f in os.listdir('/dev') if re.match(r'i2c*', f)]
bus = [i for i in Allbuses if i not in ['i2c-1','i2c-2','i2c-4']]
print("ADS i2c bus : "+str(int(bus[0].split('-')[1])))
ads_bus = I2C(int(bus[0].split('-')[1]))
ads_addr = 0x48
BATT_VOLT_DIV_RATIO = (3.3 + 10) / 3.3  # R1 = 10kOhm; R2 = 3.3kOhm

# Create the I2C bus interface.
ads = ADS.ADS1115(ads_bus, address=ads_addr)


while True:
    batt_voltage = AnalogIn(ads, ADS.P3)

    voltage = batt_voltage.voltage * BATT_VOLT_DIV_RATIO
    cap = capP(voltage)

    with open('/home/pi/MeltStake/BatteryCapacitance.txt', "w") as f:  
        # write new data line
        f.write(f"{cap:.1f}")
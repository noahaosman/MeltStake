
# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This advanced example can be used to compute a more precise reference_clock_speed. Use an
# oscilloscope or logic analyzer to measure the signal frequency and type the results into the
# prompts. At the end it'll give you a more precise value around 25 mhz for your reference clock
# speed.

# Doesnt really seem to work as is.
# A more precise way to tune the clock speed is to enter the following settings on oscilloscope connected to channel 1:
#  software package: InstrumentStudio from National Instrument
#   - time per division : 100 us
#   - x position : 1.5 ms
#   - Trigger    : edge
#   - slope      : rising
#   - level      : 1V
# Then change Clock_speed.json value and run meltstake code. Repeat until falling edge of pwm signal is centered on 1.5 ms
#
# UNPLUG ALL MOTORS BEFORE TUNING!

import meltstake as ms
import time
import json

print(ms.PCA.prescale_reg)

ms.PCA.reference_clock_speed = 25000000
ms.PCA.frequency = 100

input("Press enter when ready to measure true frequency.")

# Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
# but the PCA9685 will only actually give 12 bits of resolution.
print("Running with default calibration")
ms.PCA.channels[0].duty_cycle = 0x7FFF
time.sleep(2)
ms.PCA.channels[0].duty_cycle = 0

measured_frequency = float(input("Frequency measured: "))
print()

ms.PCA.reference_clock_speed = ms.PCA.reference_clock_speed * (
    measured_frequency / ms.PCA.frequency
)
# Set frequency again so we can get closer. Reading it back will produce the real value.
ms.PCA.frequency = 100

input("Press enter when ready to measure coarse calibration frequency.")
ms.PCA.channels[0].duty_cycle = 0x7FFF
time.sleep(2)
ms.PCA.channels[0].duty_cycle = 0
measured_after_calibration = float(input("Frequency measured: "))
print()

reference_clock_speed = measured_after_calibration * 4096 * ms.PCA.prescale_reg
print(ms.PCA.prescale_reg)
print("Real reference clock speed: {0:.0f}".format(reference_clock_speed))

ms.CLK_SPD_dict[ms.DEV_NO] = reference_clock_speed

with open("/home/pi/MeltStake/Clock_Speed.json", "w") as write_file:
    json.dump(ms.CLK_SPD_dict, write_file)

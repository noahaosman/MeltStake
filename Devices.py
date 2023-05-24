# pyright: reportMissingImports=false
import busio
import board
from adafruit_extended_bus import ExtendedI2C as I2C
import time
from datetime import datetime, timezone
import numpy as np
from threading import Thread
from adafruit_pca9685 import PCA9685  # PCA9685 module (PWM driver)
import adafruit_ads1x15.ads1115 as ADS  # ADS1115 module (ADC)
from adafruit_ads1x15.analog_in import AnalogIn
from digitalio import DigitalInOut, Direction, Pull  # GPIO module


# board V2
# ads_bus = busio.I2C(board.SCL, board.SDA)
# ads_addr = 0x49
# CURR_DRAW_DIV_RATIO = (10 + 4.7) / 10  # R1 = 4.7kOhm; R2 = 10kOhm
# BATT_VOLT_DIV_RATIO = (1 + 5.1) / 1  # R1 = 5.1kOhm; R2 = 1kOhm

# board V3
ads_bus = I2C(6)
ads_addr = 0x48
CURR_DRAW_DIV_RATIO = 1  # no volt divider
BATT_VOLT_DIV_RATIO = (3.3 + 10) / 3.3  # R1 = 10kOhm; R2 = 3.3kOhm


i2c_bus4 = I2C(4)
PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT
PWM_OE.value = False  # armed

class ADC:

    def __init__(
        self,
        over_current_pause_time=10,  # time between drill attempts
        current_limit=13,  # 13
        voltage_limit=13.5
    ):
        self.over_current_pause_time = over_current_pause_time
        self.current_limit = current_limit
        self.voltage_limit = voltage_limit
        self.under_voltage = False

        # Create the I2C bus interface.
        self.ads = ADS.ADS1115(ads_bus, address=ads_addr)

    def MonitorVoltageCurrent(self, motor_no, motors):  # to be ran as 1 thread
        # initialize:
        current_sensor = []
        current_offset = []
        self.over_current = []
        self.current = []
        for i in range(motor_no):
            current_sensor.append(AnalogIn(self.ads, eval("ADS.P"+str(i))))
            current_offset.append(10 * (2.5 - current_sensor[i].voltage * CURR_DRAW_DIV_RATIO))
            self.over_current.append(False)
            self.current.append(-1)
        batt_voltage = AnalogIn(self.ads, ADS.P3)

        while True:
            time.sleep(0.05)
            # measure current:
            for i in range(motor_no):
                self.current[i] = 10 * (2.5 - current_sensor[i].voltage * CURR_DRAW_DIV_RATIO) - current_offset[i]
                if self.current[i] > self.current_limit:
                    self.over_current[i] = True
                    Thread(daemon=True, target=motors[i].PAUSE, args=(self.over_current_pause_time,)).start()
                else:
                    self.over_current[i] = False
            # measure voltage:
            self.voltage = batt_voltage.voltage * BATT_VOLT_DIV_RATIO
            if self.voltage < self.voltage_limit:
                self.under_voltage = True
            else:
                self.under_voltage = False


class Motor:

    def __init__(
        self,
        motor_no,
        del_perc_speed=100  # avg. change in % speed per second
    ):
        self.motor_no = motor_no
        self.del_perc_speed = del_perc_speed
        self.time_between_steps = 0.05
        self.dx = 0
        self.x = 0
        self.current_speed = 0
        self.initial_speed = 0
        self.target_speed = 0
        self.pulses = 0
        self.paused = False

        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(i2c_bus4)
        self.pca.reference_clock_speed = 24350000  # Set the PWM frequency. Default 25000000
        self.pca.frequency = 200  # Set the PWM duty cycle.

    def ARMED(self):  # to be ran as 1 thread per motor
        # If we have a relay switch between battery and ESC close that switch here.
        while True:
            self.x = self.bound(self.x + self.dx)
            self.current_speed = self.easeinout()
            self.write_thrust(self.current_speed)
            time.sleep(self.time_between_steps)

    def ChangeSpeed(self, perc_in, smoothed=True):
        self.initial_speed = self.current_speed
        self.target_speed = self.bound(perc_in, -1, 1)
        if smoothed and self.initial_speed != self.target_speed: 
            total_time = abs(self.initial_speed - self.target_speed) / (self.del_perc_speed / 100)
            self.dx = 1/(total_time / self.time_between_steps)
        else:
            self.dx = 1
        self.x = 0
    
    def PAUSE(self, time_to_pause):
        self.paused = True
        print("Current threshold reached. Pausing motor "+str(self.motor_no)+" for "+str(time_to_pause)+"s")
        start_time = time.time()
        wait_time = time.time() - start_time
        while wait_time < time_to_pause and self.paused:
            self.OFF()
            wait_time = time.time() - start_time
            time.sleep(0.01)
        self.paused = False

    def OFF(self):
        self.ChangeSpeed(float(0), smoothed=False)

    def easeinout(self):
        miny = self.initial_speed
        maxy = self.target_speed
        return (maxy - miny) * (1 - np.cos(3.14 * self.x)) / 2 + miny

    def write_thrust(self, perc_speed):  # perc_speed elem [-1,1]
        pwm_input = 1500+400*perc_speed
        self.pca.channels[self.motor_no].duty_cycle = \
            int(self.pca.frequency*(10**-6)*pwm_input*65535)

    def bound(self, inp, lwr=0, upr=1):
        return max(min(upr, inp), lwr)
    
    def count_pulses(self):
        if self.motor_no == 0:
            pin = DigitalInOut(board.D5)
        else:
            pin = DigitalInOut(board.D4)#11
        pin.direction = Direction.INPUT
        pin.pull = Pull.DOWN

        prior_pin_state = 1
        while True:
            pin_state = pin.value
            if pin_state == 0 and prior_pin_state == 1:
                self.pulses = self.pulses + 1
                # time.sleep(0.05)  # 0.05s debounce timer
                print("motor "+str(self.motor_no)+" rotations :: "+str(self.pulses))
            prior_pin_state = pin_state
            time.sleep(0.01)
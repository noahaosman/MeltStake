# pyright: reportMissingImports=false
import os
import re
from Clock_Speed import CLK_SPD
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
from adafruit_extended_bus import ExtendedI2C as I2C
import time
import numpy as np
from threading import Thread
from adafruit_pca9685 import PCA9685  # PCA9685 module (PWM driver)
import adafruit_ads1x15.ads1115 as ADS  # ADS1115 module (ADC)
from adafruit_ads1x15.analog_in import AnalogIn
from math import sin, cos, asin, atan2, sqrt, pi
import socket
from icm20602 import ICM20602
from mmc5983 import MMC5983
import logging
import traceback

logging.basicConfig(level=logging.DEBUG, filename="/home/pi/data/meltstake.log", filemode="a+",
                    format="%(asctime)-15s %(levelname)-8s %(message)s")

i2c_bus4 = I2C(4)
PWM_OE = DigitalInOut(board.D26)
PWM_OE.direction = Direction.OUTPUT
PWM_OE.value = True  # disarmed
# delay arming motors in order to prevent jitter on startup.
def delay_arming():
    time.sleep(3)
    PWM_OE.value = False  # armed
Thread(target=delay_arming).start()


# get device number from static IP
try:
    f = open("/etc/dhcpcd.conf", "r")
    sstrn = 'static ip_address=10.0.1.1'
    for line in f:
        if line.startswith(sstrn):
            devno = re.split(sstrn+'|/',  line)[1]
except Exception:
    devno = '00'
# set clock speed
if devno in CLK_SPD:
    CLK_SPD = CLK_SPD[devno]
else:
    CLK_SPD = CLK_SPD['00']

class LeakDetection:

    def __init__(
        self,
    ):
        pass

    def Monitor(self):
         
        while True:
            with open('/home/pi/MeltStake/LeakState.txt', "r") as f: 
                self.State = eval(f.readline())
            time.sleep(0.25)


class ADC:

    def __init__(
        self,
        over_current_pause_time=1,  # time between drill attempts
        current_limit=14,
        voltage_limit=13.5
    ):
        self.over_current_pause_time = over_current_pause_time
        self.current_limit = current_limit
        self.voltage_limit = voltage_limit
        self.under_voltage = False

        ads_bus = I2C(22)
        ads_addr = 0x48
        self.CURR_DRAW_DIV_RATIO = 1  # no volt divider
        self.BATT_VOLT_DIV_RATIO = (3.3 + 10) / 3.3  # R1 = 10kOhm; R2 = 3.3kOhm

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
            current_offset.append(10 * (2.5 - current_sensor[i].voltage * self.CURR_DRAW_DIV_RATIO))
            self.over_current.append(False)
            self.current.append(-1)
        batt_voltage = AnalogIn(self.ads, ADS.P3)

        while True:
            time.sleep(0.05)
            # measure current:
            for i in range(motor_no):
                self.current[i] = abs(10 * (2.5 - current_sensor[i].voltage * self.CURR_DRAW_DIV_RATIO) \
                    - current_offset[i])
                if self.current[i] > self.current_limit:
                    self.over_current[i] = True
                    Thread(daemon=True, target=motors[i].PAUSE, args=(self.over_current_pause_time,)).start()
                else:
                    self.over_current[i] = False
            # measure voltage:
            self.voltage = batt_voltage.voltage * self.BATT_VOLT_DIV_RATIO
            if self.voltage < self.voltage_limit:
                self.under_voltage = True
            else:
                self.under_voltage = False
        return


class Motor:

    def __init__(
        self,
        motor_no,
        del_perc_speed=100  # avg. change in % speed per second (dec for more smoothing)
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
        self.overdrawn = False

        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(i2c_bus4)
        self.pca.reference_clock_speed = CLK_SPD  # Set the PWM frequency (Default 25000000)
        self.pca.frequency = 200  # Set the PWM duty cycle.

    def ARMED(self):  # to be ran as 1 thread per motor
        # If we have a relay switch between battery and ESC close that switch here.
        while True:
            self.x = self.bound(self.x + self.dx)
            self.current_speed = self.easeinout()
            self.write_thrust(self.current_speed)
            time.sleep(self.time_between_steps)
        return

    def ChangeSpeed(self, perc_in, smoothed=True):
        self.initial_speed = self.current_speed
        self.target_speed = self.bound(perc_in, -1, 1)
        if smoothed and self.initial_speed != self.target_speed: 
            total_time = abs(self.initial_speed - self.target_speed) / (self.del_perc_speed / 100)
            self.dx = 1/(total_time / self.time_between_steps)
        else:
            self.dx = 1
        self.x = 0
        return
    
    def PAUSE(self, time_to_pause):
        self.overdrawn = True
        logging.info("Current threshold reached. Turning off motor "+str(self.motor_no))
        start_time = time.time()
        wait_time = time.time() - start_time
        while wait_time < time_to_pause:
            wait_time = time.time() - start_time
            time.sleep(0.01)
        self.overdrawn = False

    def OFF(self):
        self.ChangeSpeed(float(0), smoothed=False)
        return

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
        # debounce code based on debounce.c written by Kenneth A. Kuhn

        DEBOUNCE_TIME = 0.003
        SAMPLE_FREQUENCY = 1000
        MAXIMUM = DEBOUNCE_TIME * SAMPLE_FREQUENCY

        integrator = 0
        output = 0
        prior_output = 0

        if self.motor_no == 0:
            pin = DigitalInOut(board.D4)
        else:
            pin = DigitalInOut(board.D8)
        pin.direction = Direction.INPUT
        pin.pull = Pull.DOWN


        while True:
            time.sleep(1/SAMPLE_FREQUENCY)

            input = pin.value

            if input == 0:
                if integrator > 0:
                    integrator = integrator - 1
            elif integrator < MAXIMUM:
                integrator = integrator + 1

            if integrator == 0:
                output = 0
            elif integrator >= MAXIMUM:
                output = 1
                integrator = MAXIMUM

            if output == 0 and prior_output == 1:
                self.pulses = self.pulses + 1

            prior_output = output


class SubLight:

    def __init__(
        self,
    ):

        # Create a simple PCA9685 class instance.
        self.pca = PCA9685(i2c_bus4)
        self.pca.reference_clock_speed = CLK_SPD  # Set the PWM frequency (Default 25000000)
        self.pca.frequency = 200  # Set the PWM duty cycle.
        pwm_input = 1500  # init off
        self.pca.channels[15].duty_cycle = \
            int(self.pca.frequency*(10**-6)*pwm_input*65535)

    def AdjustBrightness(self, perc_on = 0.5):
        perc_on = max(min(1, perc_on), 0)  # bound input to [0,1]
        pwm_input = 1100+800*perc_on
        self.pca.channels[15].duty_cycle = \
            int(self.pca.frequency*(10**-6)*pwm_input*65535)
        return


class ImuMag:

    def __init__(self):

        self.icm = ICM20602()
        self.mmc = MMC5983(i2cbus=None)

    def get_imu(self):

        data = self.icm.read_all()
        acc = [data.a.x, data.a.y, data.a.z]
        # gyr = [data.g.x, data.g.y, data.g.z]

        return acc

    def get_mag(self):

        # read the data
        data = self.mmc.read_data()
        mag = [data.x, data.y, data.z]
        # mag_raw = [data.x_raw, data.y_raw, data.z_raw]

        return mag

    def cal_mag(self):

        self.mmc.calibrate()
        # cal = [self.mmc.caldata[0], self.mmc.caldata[1], self.mmc.caldata[2]]

    def process_imu(self, accRaw):
        """ Processes raw IMU data to derive pitch and roll.
        Pitch is positive when the bow lifts up.
        Roll is positive when the port side lifts up.
        """

        # desired XYZ directions (standard)
        # X - point toward bow
        # Y - point to port
        # Z - point up

        # uncomment the relevant lines depending on the IMU orientation
        accRaw[0] = -accRaw[0]  # imu x axis points to the stern
        accRaw[1] = -accRaw[1]  # imu y axis points to the starboard
        accRaw[2] = -accRaw[2]  # imu z axis points down
        try:
            # normalize the raw accelerometer data
            norm_factor = sqrt(accRaw[0] * accRaw[0] + accRaw[1] * accRaw[1] + accRaw[2] * accRaw[2])
            accXnorm = accRaw[0] / norm_factor
            accYnorm = accRaw[1] / norm_factor
            pitch = asin(accXnorm)
            roll = -asin(accYnorm/cos(pitch))
            # convert from radians to degrees
            pitch_deg = pitch * (180/pi)
            roll_deg = roll * (180/pi)
        except Exception as e:
            pitch_deg = 0
            roll_deg = 0
        return pitch_deg, roll_deg

    def process_mag(self, magRaw, pitch_deg, roll_deg):
        """ Processes raw magnetometer data to derive heading.
        Compensates for pitch and roll.
        Heading is measured clockwise in degrees from magnetic north.
        Pitch and roll are expected to be in degrees.
        *** This method is not outputting correct heading values. ***
        """

        # desired XYZ directions (standard)
        # X - point toward bow
        # Y - point to port
        # Z - point up

        # uncomment the relevant lines depending on the magnetometer orientation
        # magRaw[0] = -magRaw[0]  # mag x axis points to the stern
        magRaw[1] = -magRaw[1]  # mag y axis points to the starboard
        # magRaw[2] = -magRaw[2]  # mag z axis points down

        # convert from degrees to radians
        pitch = pitch_deg * (pi/180)
        roll = roll_deg * (pi/180)

        # compensate for the pitch and roll
        magXcomp = magRaw[0]*cos(pitch) + magRaw[2]*sin(pitch)
        magYcomp = magRaw[0]*sin(roll)*sin(pitch) + magRaw[1]*cos(roll) - magRaw[2]*sin(roll)*cos(pitch)
        # magXcomp = magRaw[0]*cos(pitch) - magRaw[2]*sin(pitch)
        # magYcomp = magRaw[0]*sin(roll)*sin(pitch) + magRaw[1]*cos(roll) + magRaw[2]*sin(roll)*cos(pitch)

        # compensated heading in radians
        heading = atan2(magYcomp, magXcomp)

        # convert from radians to degrees
        heading_deg = heading * (180/pi)

        # convert heading from math angles (degrees CCW from x) to azimuth (degrees CW from north)
        # heading_deg = 90 - heading_deg

        # keep heading in the range 0 - 360
        if heading_deg < 0:
            heading_deg += 360

        return heading_deg

    def main(self):

        self.cal_mag()
        acc = self.get_imu()
        mag = self.get_mag()

        pitch, roll = self.process_imu(acc)
        heading = self.process_mag(mag, pitch, roll)

        return pitch, roll, heading
        

class Beacon:
    

    def __init__(self):
        self.strmsg = ''

    def Receive_Message(self):  # to be ran as thread
        PORT = 3000
        HOST = '127.0.0.1'
        MAX_LENGTH = 4096
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.bind((HOST, PORT))
        serversocket.listen(5)
        while True:
            # accept connections from outside
            (clientsocket, address) = serversocket.accept()
            msg = clientsocket.recv(MAX_LENGTH)
            if msg == '':  # client terminated connection
                clientsocket.close()
            self.strmsg = msg.decode()

    def Transmit_Message(self, msg):
        try:
            HOST = '127.0.0.1'
            PORT = 4000
            s = socket.socket()
            s.connect((HOST, PORT))
            fullmsg = '106 RE: ' + msg
            s.send(fullmsg.encode())
            s.close()

            # hotfix for sending responses to laptop beacon. Should be able to just read ID of pinging beacon?
            # time.sleep(0.5) 
            # s.connect((HOST, PORT))
            # fullmsg = '203 RE: ' + msg
            # s.send(fullmsg.encode())
            # s.close()
            
            logging.info(fullmsg)
        except Exception:
            logging.info("ERROR transmitting message: " + msg)
            logging.info(traceback.format_exc())
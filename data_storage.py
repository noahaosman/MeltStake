# pyright: reportMissingImports=false
import time
from datetime import datetime, timezone
import inspect
from brping import Ping1D      
import ms5837
from Devices import ImuMag
import logging
import re
import sys, os
from contextlib import contextmanager

# useful function to suppress print statements within a function (taken from Dave Smith's blog)
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        sys.stdout = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout

logging.basicConfig(level=logging.DEBUG, filename="/home/pi/data/meltstake.log", filemode="a+",
                    format="%(asctime)-15s %(levelname)-8s %(message)s")

startup_delay = 2

class Data:
    def __init__(
        self
    ):
        self.IV=[0]
        self.ROT=[0]
        self.PING=[0]
        self.IMU=[0]
        self.PT=[0]

    def WriteToFile(self, data_list = None, data_bin = False):
        # use outer function name as data_bin if not specified
        if data_bin == False:
            cframe = inspect.currentframe()
            data_bin = inspect.getframeinfo(cframe.f_back).function
        
        # Filenaming convention: <data_bin>.dat
        dt = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        current_date = dt[0:10]
        current_time = dt[0:23]
        filename = "/home/pi/data/" + data_bin + ".dat"

        # Write new line to file. Format: current_time    data_1    ...    data_n
        if data_list != None:
            with open(filename, "a+") as f:  
                # write new data line
                f.write(current_time + "\t")

                str_data = [ f"{data_point:.3f}" if isinstance(data_point, float) else str(data_point) for data_point in data_list]
                for data_point in str_data:
                    f.write(data_point)
                    f.write("\t")
                f.write("\n")


    def CurrentDraw(self, battery, motors, sample_rate = 10):  # Format: Voltage    Current_1    ...    Current_n
        time.sleep(startup_delay)  # give some time for other threads to start up

        while True:
            time.sleep(1/sample_rate)
            self.IV = []
            self.IV.append(battery.voltage)
            for i in range(len(motors)):
                self.IV.append(battery.current[i])
            # if abs(battery.current[0])+abs(battery.current[1]) > 0.25:  # if drawing more than 250 mA, record data
            self.WriteToFile(self.IV)


    def Rotations(self, motors, sample_rate = 10):  # to be ran as thread
        time.sleep(startup_delay)  # give some time for other threads to start up

        while True:
            time.sleep(1/sample_rate)
            self.ROT = [motor.pulses for motor in motors]
            self.WriteToFile(self.ROT)


    def Ping(self, sample_rate = 10):  # Format: Distance    % confidence
        time.sleep(startup_delay)  # give some time for other threads to start up

        with suppress_stdout():   
            myPing = Ping1D()
            myPing.connect_serial("/dev/ttyAMA0", 115200)

        while myPing.initialize() is False:
            time.sleep(10)  # if ping fails to init, try again in 10 seconds

        while True:
            time.sleep(1/sample_rate)
            ping_data = myPing.get_distance()
            if ping_data:
                self.PING = [ping_data["distance"], ping_data["confidence"]]
                self.WriteToFile(self.PING)

    def Orientation(self, sample_rate = 10):  # Format: pitch    roll    heading
        time.sleep(startup_delay)  # give some time for other threads to start up

        im = ImuMag()  # initialize IMU
        with suppress_stdout():  
            self.IMU = im.main() #IMU data

        while True:
            time.sleep(1/sample_rate)
            self.IMU = im.main() #IMU data
            self.WriteToFile(self.IMU)


    def Pressure(self, sample_rate = 10):  # to be ran as thread
        time.sleep(startup_delay)  # give some time for other threads to start up

        Allbuses = [f for f in os.listdir('/dev') if re.match(r'i2c*', f)]
        bus = [i for i in Allbuses if i not in ['i2c-1','i2c-2','i2c-4']]

        with suppress_stdout(): #IMU data
            sensor = ms5837.MS5837_30BA(int(bus[0].split('-')[1]))

            if not sensor.init():  # initialize sensor
                logging.info("Pressure sensor could not be initialized")
                exit(1)
            if not sensor.read():  # Check we can read from sensor
                logging.info("Pressure sensor read failed!")
                exit(1)

        while True:
            time.sleep(1/sample_rate)
            sensor.read()
            P = sensor.pressure(ms5837.UNITS_atm)
            T = sensor.temperature(ms5837.UNITS_Centigrade)
            self.PT = [P, T]
            self.WriteToFile( self.PT )
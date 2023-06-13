# pyright: reportMissingImports=false
import time
from datetime import datetime, timezone
import os
import inspect
from brping import Ping1D      
import ms5837
from Devices import ImuMag
import logging


logging.basicConfig(level=logging.DEBUG, filename="/home/pi/data/meltstake.log", filemode="a+",
                    format="%(asctime)-15s %(levelname)-8s %(message)s")

startup_delay = 3

def WriteToFile(data_list = None, data_bin = False):
    # use outer function name as data_bin if not specified
    if data_bin == False:
        cframe = inspect.currentframe()
        data_bin = inspect.getframeinfo(cframe.f_back).function
    
    # Filenaming convention: <data_bin>.dat
    dt = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
    current_date = dt[0:10]
    current_time = dt[0:23]
    filename = "/home/" + os.getlogin() + "/data/" + data_bin + ".dat"

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


def CurrentDraw(battery, motors, sample_rate = 10):  # Format: Voltage    Current_1    ...    Current_n
    time.sleep(startup_delay)  # give some time for other threads to start up

    while True:
        time.sleep(1/sample_rate)
        IV = []
        IV.append(battery.voltage)
        for i in range(len(motors)):
            IV.append(battery.current[i])
        # if abs(battery.current[0])+abs(battery.current[1]) > 0.25:  # if drawing more than 250 mA, record data
        WriteToFile(IV)


def Rotations(motors, sample_rate = 10):  # to be ran as thread
    time.sleep(startup_delay)  # give some time for other threads to start up

    while True:
        time.sleep(1/sample_rate)
        ROT = [motor.pulses for motor in motors]
        WriteToFile(ROT)


def Ping(sample_rate = 10):  # Format: Distance    % confidence
    time.sleep(startup_delay)  # give some time for other threads to start up
    
    myPing = Ping1D()
    myPing.connect_serial("/dev/ttyAMA0", 115200)

    while True:
        time.sleep(1/sample_rate)
        ping_data = myPing.get_distance()
        if ping_data:
            PING = [ping_data["distance"], ping_data["confidence"]]
            WriteToFile(PING)

def Orientation(sample_rate = 10):  # Format: pitch    roll    heading
    time.sleep(startup_delay)  # give some time for other threads to start up
    
    im = ImuMag()  # initialize IMU

    while True:
        time.sleep(1/sample_rate)
        IMU = im.main() #IMU data
        WriteToFile(IMU)


def Pressure(sample_rate = 10):  # to be ran as thread
    time.sleep(startup_delay)  # give some time for other threads to start up

    sensor = ms5837.MS5837_30BA(6)        
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
        PT = [P, T]
        WriteToFile( PT )
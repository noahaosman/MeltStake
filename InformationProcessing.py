# pyright: reportMissingImports=false
import time
from datetime import datetime, timezone
import socket
import os
import inspect
from brping import Ping1D      
import ms5837
from imu_mag import ImuMag

class Data:


    def WriteToFile(self, data_list = "init", data_bin = False):
        # use outer function name as data_bin if not specified
        if data_bin == False:
            cframe = inspect.currentframe()
            data_bin = inspect.getframeinfo(cframe.f_back).function
        
        # Filenaming convention: <data_bin><date>.dat
        dt = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        current_date = dt[0:10]
        current_time = dt[0:23]
        filename = "/home/" + os.getlogin() + "/data/" + data_bin + ".dat"

        # Write new line to file. Format: current_time    data_1    ...    data_n
        with open(filename, "a+") as f:  
            if data_list == "init":  # initialize data file if no data is given
                f.write(current_time + "\t")
                f.write("POWER_ON\t\n")
            else:

                # Only write to file if the new data point is different than the last recorded data point
                f.seek(0)
                try:
                    last_line = f.readlines()[-1]
                except:
                    last_line = ""  # placeholder line for empty file. This only happens when date changes mid deployment.
                read_data = last_line.split("\t")[1:-1]
                str_data = [f"{data_point:.3f}" for data_point in data_list]

                # write new data line
                if str_data != read_data:  # replace this conditional with 'True' to save every data point
                    f.write(current_time + "\t")
                    for data_point in str_data:
                        f.write(data_point)
                        f.write("\t")
                    f.write("\n")


    def CurrentDraw(self, battery, motors, sample_rate = 10):  # Format: Voltage    Current_1    ...    Current_n
        self.WriteToFile()  # initialize data file
        time.sleep(5)  # give some time for other threads to start up

        while True:
            time.sleep(1/sample_rate)
            self.IV = []
            self.IV.append(battery.voltage)
            for i in range(len(motors)):
                self.IV.append(battery.current[i])
            if abs(battery.current[0])+abs(battery.current[1]) > 0.25:  # if drawing more than 250 mA, record data
                self.WriteToFile(self.IV)


    def Rotations(self, motors, sample_rate = 10):  # to be ran as thread
        self.WriteToFile()  # initialize data file
        time.sleep(5)  # give some time for other threads to start up

        while True:
            time.sleep(1/sample_rate)
            self.ROT = [motors[motor_no].pulses for motor_no in range(len(motors))]
            self.WriteToFile(self.ROT)


    def Ping(self, sample_rate = 10):  # Format: Distance    % confidence
        self.WriteToFile()  # initialize data file
        time.sleep(5)  # give some time for other threads to start up
        
        myPing = Ping1D()
        myPing.connect_serial("/dev/ttyAMA0", 115200)

        while True:
            time.sleep(1/sample_rate)
            ping_data = myPing.get_distance()
            if ping_data:
                self.PING = [ping_data["distance"], ping_data["confidence"]]
                self.WriteToFile(self.PING)

    def Orientation(self, sample_rate = 10):  # to be ran as thread
        self.WriteToFile()  # initialize data file
        time.sleep(5)  # give some time for other threads to start up
        
        im = ImuMag()  # initialize IMU

        while True:
            time.sleep(1/sample_rate)
            self.IMU = im.main() #IMU data
            self.WriteToFile(self.IMU)


    def Pressure(self, sample_rate = 10):  # to be ran as thread
        self.WriteToFile()  # initialize data file
        time.sleep(5)  # give some time for other threads to start up

        sensor = ms5837.MS5837_30BA(6)        
        if not sensor.init():  # initialize sensor
            print("Pressure sensor could not be initialized")
            exit(1)
        if not sensor.read():  # Check we can read from sensor
            print("Pressure sensor read failed!")
            exit(1)

        while True:
            time.sleep(1/sample_rate)
            P = sensor.pressure(ms5837.UNITS_atm)
            T = sensor.temperature(ms5837.UNITS_Centigrade)
            self.PT = [P, T]
            self.WriteToFile( self.PT )


class Comms:
    def __init__(self):
        pass
        
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
                print(fullmsg)
                s.send(fullmsg.encode())
                s.close()
            except Exception:
                print(msg)
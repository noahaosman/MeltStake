# pyright: reportMissingImports=false
import time
from threading import Thread
import numpy as np
import logging
import traceback
import ms5837
import os
import re
from datetime import datetime, timezone
logging.basicConfig(level=logging.DEBUG, filename="/home/pi/data/meltstake.log", filemode="a+",
                    format="%(asctime)-15s %(levelname)-8s %(message)s")


# This class contains all operations that can be called via acoustic beacon
class Operations:

    def __init__(self, args, motors):
        self.num_motors = len(motors)
        self.disarm = False
        if args.mode == 'debug':
            self.speed = 0.2
        else:
            self.speed = 0.6


    def DRILL(self, motors, target_turns):  
        # Power each motor for the specified # of turns. Negative for CCW, Positive for CW.
         
        # clean up input: 
        target_turns = [int(str_in) for str_in in target_turns]  # convert string input to int
        target_turns.extend([0] * (len(motors) - len(target_turns)))  # pad end with 0's if input was less than number of motors
        target_turns = target_turns[0:len(motors)]  # remove extra elements if larger than the number of motors

        # initial number of turns
        turns = [0]*len(motors)

        # determine direction(s) to move
        directions = [np.sign(goal-current) for goal,current in zip(target_turns,turns)]

        # intialize flag conditional
        target_reached = [False]*len(motors)

        self.disarm = False
        while any([not done for done in target_reached]) and not self.disarm:

            # update starting number of turns
            offsets = [motor.pulses for motor in motors]  # initial # of pulses

            # adjust speed for each motor:
            for motor_no, (motor, dir, done, targ, curr) in enumerate(zip(motors, directions, target_reached, target_turns, turns)):
                if not done:
                    if dir * (targ - curr) <= 0 or motor.overdrawn:  # if target is reached, or if overcurrent 
                        target_reached[motor_no] = True
                        if abs(motor.current_speed) > 0 and abs(motor.current_speed) < 0.3:
                            time.sleep(0.1) # give some extra time to move magnet away from reed switch
                        motor.ChangeSpeed(0, smoothed=False)  # turn motors off=
                    else:
                        motor.ChangeSpeed(dir * self.speed, smoothed=True)  # update motor speed

            time.sleep(0.05)  # give some time for system to move

            # determine number of signed rotations since last iteration
            change_in_turns = [np.sign(motor.current_speed) * (motor.pulses - offset) for motor, offset in zip(motors,offsets)]
            turns = [int(sum(x)) for x in zip(turns, change_in_turns)]  # update turn counter

        # Once we reach our target turns, set all speeds to zero and break
        self.OFF(motors)
        return

    def RELEASE(self, motors, arguments=None):  
        # release unit from ice (note: approx 36 rotations for length of ice screw)
        # this function will first read most recent pressure measurements, then if both
        #    1) depth is greater than 0.5 meters
        #    2) rate of change of depth is less than 0.1 m/s (i.e. meltstake is not rising)
        # it will try to release 10 times, then give up.

        def read_n_to_last_line(filename, n = 1):
            """Returns the nth before last line of a file (n=1 gives last line)"""
            num_newlines = 0
            with open(filename, 'rb') as f:
                try:
                    f.seek(-2, os.SEEK_END)    
                    while num_newlines < n:
                        f.seek(-2, os.SEEK_CUR)
                        if f.read(1) == b'\n':
                            num_newlines += 1
                except OSError:
                    f.seek(0)
                last_line = f.readline().decode()
            return last_line
        
        self.OFF(motors)

        attempts = 0
        while attempts <= 10:

            lastPreads = [[None,None],[None,None]]
            for i in [0,1]:
                data = read_n_to_last_line('/home/pi/data/Pressure.dat', n = i*2+2)
                data = data.split()
                lastPreads[i][0] = datetime.strptime(data[0], '%Y-%m-%dT%H:%M:%S.%f')
                lastPreads[i][1] = float(data[1])
                print(data)

            dt = datetime.now()
            time_since_last_Pread = (dt - lastPreads[1][0]).total_seconds()
            time_between_Preads = (lastPreads[0][0] - lastPreads[1][0]).total_seconds()

            if time_since_last_Pread > 1 or time_between_Preads > 5:
                # bad pressure reading, assume we're at depth
                Pread = False
                depth = 2
                velocity = 0
                logging.info("Bad pressure reading")
            else:
                Pread = True
                P0 = lastPreads[1][1]
                P1 = lastPreads[0][1]
                depth = (P0+P1)/2
                velocity = (P1-P0)/time_between_Preads

            # if we're in the ice, try to drill out
            if ((depth > 1.05 and velocity < 0.001) or not Pread): 
                logging.info("RELEASE loop "+str(attempts+1)+"/10")
                self.DRILL(motors, [-50, -50])
                if attempts > 0: # try to reverse in if it seems like we're stuck
                    self.DRILL(motors, [5, 5])
            else:
                break

            attempts = attempts + 1
        return
    
    def OFF(self, motors):  
        # set all motors to zero speed
        self.disarm = True
        # turn motors off
        for motor in motors:
            motor.OFF()
        return

    def SETROT(self, motors, set_turns):  
        # maunally overwrite rotation tracking number
        
        # clean up input:
        set_turns = [int(str_in) for str_in in set_turns]  # convert string input to int
        set_turns.extend([None] * (len(motors) - len(set_turns)))  # pad end with None's if input was less than number of motors
        set_turns = set_turns[0:len(motors)]  # remove extra elements if larger than the number of motors

        for motor, set_turn in motors, set_turns:
            if set_turn != None:
                motor.pulses = set_turn

    
    def SETSPD(self, motors, arguments=None):
        flt_spd = [float(str_spd) for str_spd in arguments]
        flt_spd = flt_spd[0]
        if flt_spd is not None and flt_spd > 0 and flt_spd < 1:
            self.speed = flt_spd



    def DATA(self, data, beacon, arguments=None):
        # send most recent data measurement via beacon tx
        # arguement options:
        #   IV     ::  current, voltage
        #   ROT    ::  rotations
        #   PING   ::  ping sonar
        #   IMU    ::  pitch, tilt, roll (?)
        #   PT     ::  pressure, temperature from ms5837

        for data_req in arguments:
            logging.info("Attempting to transmit "+data_req+" data ... ")
            time.sleep(1)
            try:
                data_list = eval("data."+data_req)  # get most recent measurement
                str_dat = [ f"{data_point:.3f}" if isinstance(data_point, float) else str(data_point) \
                           for data_point in data_list]
                msg = data_req + " " + ' '.join(str_dat)
                beacon.Transmit_Message(msg)  # transmit requested data
            except Exception as e:
                logging.info("Data transmission failed")
                logging.info(traceback.format_exc())
                pass

        return


# pyright: reportMissingImports=false
import time
from threading import Thread
import numpy as np
import logging
import ms5837
import traceback
import os
import re
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
        # this function will first take a measure of the pressure, then if both
        #    1) depth is greater than 0.5 meters
        #    2) rate of change of depth is less than 0.1 m/s (i.e. meltstake is not rising)
        # it will try to release 10 times, then give up.
        self.OFF(motors)

        Allbuses = [f for f in os.listdir('/dev') if re.match(r'i2c*', f)]
        bus = [i for i in Allbuses if i not in ['i2c-1','i2c-2','i2c-4']]
        PTsensor = ms5837.MS5837_30BA(int(bus[0].split('-')[1]))  

        if not PTsensor.init():
            Pread = False
            P0 = 1
            P1 = 1
        else:
            Pread = True
            PTsensor.read()
            P0 = PTsensor.pressure(ms5837.UNITS_atm)
            time.sleep(0.25)
            PTsensor.read()
            P1 = PTsensor.pressure(ms5837.UNITS_atm)

        attempts = 0
        while ((P1 > 1.05 and (P1-P0)/0.25 < 0.001) or not Pread) and attempts <= 10: 
            self.DRILL(motors, [-50, -50])
            time.sleep(1)
            self.DRILL(motors, [3, 3])
            time.sleep(1)
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


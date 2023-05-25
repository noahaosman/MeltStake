import time
from threading import Thread
import numpy as np


# This class contains all operations that can be called via acoustic beacon
class Operations:

    def __init__(self, args, motors):
        self.num_motors = len(motors)
        self.disarm = False
        if args.mode == 'debug':
            self.speed = 0.2
        else:
            self.speed = 0.8


    def DRILL(self, motors, target_turns):  # Power each motor for the specified # of turns. Negative for CCW, Positive for CW.
        
        target_turns = [int(str_in) for str_in in target_turns]  # convert string input to int
        target_turns.extend([0] * (len(motors) - len(target_turns)))  # pad end with 0's if input was less than number of motors
        target_turns = target_turns[0:len(motors)]  # remove extra elements if larger than the number of motors
        # TODO: make this less dumb ^

        turns = [0]*len(motors)

        self.disarm = False
        while any([a!=b for a,b in zip(turns,target_turns)]) and not self.disarm:  # keep looping as long as we have turns left to make
            
            directions = [np.sign(goal-current) for goal,current in zip(target_turns,turns)]  # determine directions we need to move
            offsets = [motor.pulses for motor in motors]  # initial # of pulses

            # adjust speed for each motor:
            for motor, direction in zip(motors, directions):
                motor.ChangeSpeed(direction * self.speed, smoothed=True)  # update motor speed
            
            time.sleep(0.1)  # give some time for system to move

            # determine number of signed rotations since last iteration
            change_in_turns = [np.sign(motor.current_speed) * (motor.pulses - offset) for motor, offset in zip(motors,offsets)]
            turns = [int(sum(x)) for x in zip(turns, change_in_turns)]  # update turn counter
        

        # Once we reach our target turns, set all speeds to zero and break
        self.OFF(motors)
        return

    def RELEASE(self, motors, arguments=None):  # release unit from ice face (approx 36 rotations to move length of ice screw)
        self.DRILL(motors, [-50, -50])
    
    def OFF(self, motors):  # release unit from ice face
        self.disarm = True
        # turn motors off
        for motor in motors:
            motor.OFF()
        return

    def DATA(self, data, beacon, arguments=None):
        # arguement options:
        #   IV     ::  current, voltage
        #   ROT    ::  rotations
        #   PING   ::  ping sonar
        #   IMU    ::  pitch, tilt, roll (?)
        #   PT     ::  pressure, temperature from ms5837

        for data_req in arguments:
            print("Attemptng to transmit "+data_req+" data ... ")
            try:
                dat = eval("data."+data_req)
                msg = data_req + str(dat)
                print(msg)
                # transmit requested data
                beacon.Transmit_Message(msg)
            except Exception:
                print("Data transmission failed")
                pass

        return


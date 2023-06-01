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


    def DRILL(self, motors, target_turns):  
        # Power each motor for the specified # of turns. Negative for CCW, Positive for CW.
        
        target_turns = [int(str_in) for str_in in target_turns]  # convert string input to int
        target_turns.extend([0] * (len(motors) - len(target_turns)))  # pad end with 0's if input was less than number of motors
        target_turns = target_turns[0:len(motors)]  # remove extra elements if larger than the number of motors
        # TODO: make this less dumb ^

        turns = [0]*len(motors)

        directions = [np.sign(goal-current) for goal,current in zip(target_turns,turns)]  # determine directions to move

        unhappy = [True, True]

        self.disarm = False
        while any(unhappy) and not self.disarm:
            
            offsets = [motor.pulses for motor in motors]  # initial # of pulses

            # adjust speed for each motor:
            for motor, direction, motor_no in zip(motors, directions, range(len(motors))):
                if unhappy[motor_no]:
                    if direction < 0:
                        if turns <= target_turns:
                            unhappy[motor_no] = False
                            motor.ChangeSpeed(0, smoothed=False)  # update motor speed
                        else:
                            motor.ChangeSpeed(direction * self.speed, smoothed=True)  # update motor speed
                    if direction > 0:
                        if turns >= target_turns:
                            unhappy[motor_no] = False
                            motor.ChangeSpeed(0, smoothed=False)  # update motor speed
                        else:
                            motor.ChangeSpeed(direction * self.speed, smoothed=True)  # update motor speed
                    if direction == 0:
                        unhappy[motor_no] = False


            time.sleep(0.05)  # give some time for system to move

            # determine number of signed rotations since last iteration
            change_in_turns = [np.sign(motor.current_speed) * (motor.pulses - offset) \
                               for motor, offset in zip(motors,offsets)]
            turns = [int(sum(x)) for x in zip(turns, change_in_turns)]  # update turn counter

        # Once we reach our target turns, set all speeds to zero and break
        self.OFF(motors)
        return

    def RELEASE(self, motors, arguments=None):  
        # release unit from ice (~36 rotations for length of ice screw)
        self.OFF(motors)
        self.DRILL(motors, [-50, -50, -50])
        return
    
    def OFF(self, motors):  
        # set all motors to zero speed
        self.disarm = True
        # turn motors off
        for motor in motors:
            motor.OFF()
        return

    def SETROT(self, motors, set_turns):  
        # Power each motor for the specified # of turns. Negative for CCW, Positive for CW.
        
        set_turns = [int(str_in) for str_in in set_turns]  # convert string input to int
        set_turns.extend([-1] * (len(motors) - len(set_turns)))  # pad end with -1's if input was less than number of motors
        set_turns = set_turns[0:len(motors)]  # remove extra elements if larger than the number of motors
        # TODO: make this less dumb ^

        for motor, set_turn in motors, set_turns:
            if set_turn != -1:
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
            print("Attemptng to transmit "+data_req+" data ... ")
            time.sleep(1)
            try:
                data_list = eval("data."+data_req)  # get most recent measurement
                str_dat = [ f"{data_point:.3f}" if isinstance(data_point, float) else str(data_point) \
                           for data_point in data_list]
                msg = data_req + " " + ' '.join(str_dat)
                beacon.Transmit_Message(msg)  # transmit requested data
            except Exception:
                print("Data transmission failed")
                pass

        return


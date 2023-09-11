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

    def __init__(self, mode, motors):
        self.num_motors = len(motors)
        self.disarm = False
        if mode == 'debug':
            self.speed = 0.2
        else:
            self.speed = 0.6
        self.SOS_flag = False


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
                        motor.ChangeSpeed(0, smoothed=False)  # turn motors off
                    else:
                        motor.ChangeSpeed(dir * self.speed, smoothed=True)  # update motor speed

            time.sleep(0.05)  # give some time for system to move

            # determine number of signed rotations since last iteration
            change_in_turns = [np.sign(motor.current_speed) * (motor.pulses - offset) for motor, offset in zip(motors,offsets)]
            turns = [int(sum(x)) for x in zip(turns, change_in_turns)]  # update turn counter

        # Once we reach our target turns, set all speeds to zero and break
        self.OFF(motors)
        return
    
    def AUTONOMOUS(self, motors, deployment_time):
        # operation for autonomous deployment
        # try to drill in x rotations every y minutes

        def deployment_timer(deployment_time):

            while True:
                time.sleep(1)

                if (time.time()-init_time) > (deployment_time*3600):
                    break

            return

        rotations_per_drill = 20
        time_between_drills = 20 # minutes

        self.OFF(motors)

        # tare rotation tracker
        self.SETROT(motors, [0,0])

        init_time = time.time()
        last_drill_time = init_time

        while (time.time()-init_time) > (deployment_time*3600) and not self.SOS_flag:

            time.sleep(0.1)

            # wait specified time between drill attempts
            if (time.time()-last_drill_time) > (time_between_drills*60): 
                # try drilling in. 
                # For both screws this will either do the full 20 rotations or over-current out
                Thread(daemon=True, target=self.DRILL, args=(motors, [rotations_per_drill]*2 )).start()
                last_drill_time = time.time()
                logging.info("AUTONOMOUS DRILLING")


        self.OFF(motors)

        return

    def RELEASE(self, motors, arguments=None):  
        # release unit from ice (note: approx 36 rotations for length of ice screw)
        # this function will first read most recent pressure measurements, then if
        # depth is greater than 0.5 meters
        # If number of rotations is not increasing (ie screws are stuck) try
        # drilling into ice 5 rotations, then back out. (this can help loosen ice)

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

        def get_saved_data(data_type, time_between=0):
            # get last 2 measurements
            reads = [[None,None,None],[None,None,None]]
            for i in [0,1]:
                data = read_n_to_last_line("/home/pi/data/"+data_type+".dat", n = i*2 + 2 + i*10*time_between)
                data = data.split()
                reads[i][0] = datetime.strptime(data[0], '%Y-%m-%dT%H:%M:%S.%f')
                for j in range(1,len(data)):
                    reads[i][j] = float(data[j])

            dt = datetime.now()
            time_since_last_read = (dt - reads[1][0]).total_seconds()
            time_between_reads = (reads[0][0] - reads[1][0]).total_seconds()

            data_read = True
            if data_type == "Pressure":
                if time_since_last_read > 1 or time_between_reads > 5:
                    # bad reading
                    data_read = False
                    out1 = 0
                    out2 = 0
                    logging.info("Bad "+data_type+" reading")
                else:
                    P0 = reads[1][1]
                    P1 = reads[0][1]
                    out1 = (P0+P1)/2 #depth
                    out2 = (P1-P0)/time_between_reads #velocity
            if data_type == "Rotations":
                out1 = reads[1][1] - reads[0][1]# difference in encoder 1
                out2 = reads[1][2] - reads[0][2]# difference in encoder 2

            return [data_read, out1, out2]

        def check_if_floating():
            self.stuck = True
            while True:
                [Pread, depth, velocity] = get_saved_data("Pressure")
                if Pread and (depth <= 1.05):# or velocity > 0.001): 
                    self.stuck = False 
                    break
                time.sleep(0.5)
            return
        Thread(daemon=True, target=check_if_floating).start()
        time.sleep(0.1)

        loops = 0
        in_attempts = -1
        wait_time = 1
        self.OFF(motors)
        Thread(daemon=True, target=self.DRILL, args=(motors, [-1000, -1000] )).start()
        while True:
            
            # if we're below the surface, try to drill out
            if self.stuck: 
                
                time.sleep(wait_time)

                try:
                    # every 20 seconds check if # of rotations are increasing:
                    if loops*wait_time > 20: 
                        loops = 0
                        [Rread, rotdot0, rotdot1] = get_saved_data("Rotations", 2)
                        if (rotdot0 == 0 or rotdot1 == 0) or not Rread: #if either stake is stuck
                            # attempt to drill in 5 turns (this sometimes helps loosen the ice)
                            # alternate between left, right, and both drilling in
                            in_attempts = (in_attempts + 1) % 3

                            if in_attempts == 1:
                                drill_in = [5, 0]
                            elif in_attempts == 2:
                                drill_in = [0, 5]
                            else:
                                drill_in = [5, 5]

                            self.OFF(motors) # stop drill out
                            time.sleep(0.5)
                            Thread(daemon=True, target=self.DRILL, args=(motors, drill_in)).start() # start drill in
                            time.sleep(5)
                            self.OFF(motors) # stop drill in
                            time.sleep(0.5)
                            Thread(daemon=True, target=self.DRILL, args=(motors, [-1000, -1000] )).start() # resume drill out
                except Exception:
                    pass
                
            else:
                break
            
            loops = loops+1

        logging.info("exiting RELEASE operation")

        self.OFF(motors)
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


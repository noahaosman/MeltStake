# pyright: reportMissingImports=false
import logging
import time
import argparse
import numpy as np
from threading import Thread
import os
import subprocess
import re

import Devices
import data_storage as data
from Operations import Operations


if __name__ == "__main__":

    # assign log file
    logging.basicConfig(level=logging.DEBUG, filename="/home/pi/data/meltstake.log", filemode="a+",
                        format="%(asctime)-15s %(levelname)-8s %(message)s")

    # parse arguements
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-m", "--mode", help=" mode of operation. Options: debug, deploy", default='deploy')
    args = argParser.parse_args()

    # Initialize classes
    battery = Devices.ADC()
    motors = [Devices.Motor(0), Devices.Motor(1)]  # Hardware supports up to 3 motors
    light = Devices.SubLight()

    print("opening threads...")
    t_ARMED = []
    t_count_pulses = []
    for i in range(len(motors)):
        Thread(daemon=True, target=motors[i].ARMED).start()
        Thread(daemon=True, target=motors[i].count_pulses).start()
    Thread(daemon=True, target=battery.MonitorVoltageCurrent, args=(len(motors), motors, )).start()
    light.AdjustBrightness(0)  # turn on sub light

    # SAVE DATA:
    SAMPLE_RATE = 10  # data sampling rate in Hz
    Thread(daemon=True, target=data.CurrentDraw, args=(battery, motors, SAMPLE_RATE,)).start()
    Thread(daemon=True, target=data.Rotations, args=(motors, SAMPLE_RATE,)).start()
    Thread(daemon=True, target=data.Ping, args=(SAMPLE_RATE,)).start()
    Thread(daemon=True, target=data.Orientation, args=(SAMPLE_RATE,)).start()
    Thread(daemon=True, target=data.Pressure, args=(SAMPLE_RATE,)).start()

    beacon = Devices.Beacon()
    Thread(daemon=True, target=beacon.Receive_Message).start()

    commands = Operations(args, motors)
    t_operation = Thread()  # initialize main thread variable
    known_commands = [attribute for attribute in dir(commands) if \
                      callable(getattr(commands, attribute)) and attribute.startswith('__') is False]
    print(known_commands)

    time.sleep(1)

    # MAIN LOOP:
    while not battery.under_voltage:

        try:
            time.sleep(0.05)

            ### Receive/transmit beacon messages
            if args.mode == 'debug':
                beacon.strmsg = input("input: ")  # Terminal input for testing.

            if beacon.strmsg != '':

                msg = beacon.strmsg
                beacon.Transmit_Message(msg)  # echo back the message
                
                msg_split = msg.split()
                command = msg_split[0]
                arguments = [msg_split[i] for i in range(1,len(msg_split))]

                if command == 'OFF':
                    commands.OFF(motors)

                elif command == 'DATA':
                    commands.DATA(data, beacon, arguments)

                elif command == 'LIGHT':
                    flt_in = float(arguments[0])
                    light.AdjustBrightness(flt_in/100)

                elif command in known_commands:  # any other commands will begin as a thread
                    t_new = Thread(daemon=True, target=eval("commands."+command), args=(motors, arguments, ))
                    
                    if not t_operation.is_alive():  # don't overwrite any currently running operations
                        t_operation = t_new
                        t_operation.start()
                    else:
                        beacon.Transmit_Message("BUSY -- SEND OFF TO CLEAR")
                        print("operation currently running... send 'OFF' to kill")
                
                beacon.strmsg = ''


        except Exception as e:
            logging.info("--- RUNTIME ERROR: ---")
            logging.info(e)
            if args.mode != 'debug':
                commands.RELEASE(motors)  # release device from ice face
            break

    if battery.under_voltage:
        logging.info("LOW BATTERY! :: "+ str(battery.voltage/battery.BATT_VOLT_DIV_RATIO))
        if args.mode == 'debug':
            for i in range(len(motors)):
                motors[i].OFF()  # set all motors to off before exiting code
        else:
            commands.RELEASE(motors)  # release device from ice face
    
    light.AdjustBrightness(0.0)


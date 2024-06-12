#!/usr/bin/env python3
# pyright: reportMissingImports=false
import os
import time
import argparse
import traceback
import logging
import time
import argparse
from threading import Thread
import traceback
import Operations
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module


Operations.OFF()
Operations.AR_OVRD('T')
Operations.max_speed = 0.2

armed = False

def button():

    pin = DigitalInOut(board.D4)
    pin.direction = Direction.INPUT
    pin.pull = Pull.DOWN
    
    state = 'OFF'
    while armed:
        input = pin.value
        if input == 0 and state == 'OFF':
            Thread(daemon=True, target=Operations.DRILL, args=([100, 100],)).start()
            state = 'ON'
        elif input == 1 and state == 'ON':
            Operations.OFF()
            state = 'OFF'
        time.sleep(0.25)
        
    return


t_operation = Thread()  # initialize main thread variable
known_commands = [attribute for attribute in dir(Operations) if \
                    callable(getattr(Operations, attribute)) and attribute.startswith('__') is False]

while not Operations.battery.under_voltage and not Operations.leaksenor.state:
    try:
        time.sleep(0.05)

        recieved_msg = input("input: ")  # Terminal input for testing.

        msg = recieved_msg.upper()

        transmit_msg = msg  # echo back the message
        
        msg_split = msg.split()
        command = msg_split[0]
        arguments = [msg_split[i] for i in range(1,len(msg_split))]
        
        
        if command == 'ARM':
            armed = True
            time.sleep(0.1)
            Thread(daemon=True, target=button).start()

        elif command == 'DISARM':
            armed = False
            Operations.OFF()

        elif command == 'OFF':
            Operations.OFF()
            Operations.stopauto = True
        
        elif command == 'STOPAUTO':
            Operations.stopauto = True
            
        elif command == 'AR_OVRD':
            Operations.AR_OVRD(arguments)
            
        elif command == 'LS_OVRD':
            Operations.LS_OVRD(arguments)

        elif command == 'CLA':
            Operations.CLA(arguments)

        elif command == 'LIGHT':
            try:
                flt_in = float(arguments[0])
                Operations.light.brightness = flt_in/100
            except:
                pass

        elif command in known_commands:  # any other commands will begin as a thread
            t_new = Thread(daemon=True, target=eval("Operations."+command), args=(arguments, ))
            
            if not t_operation.is_alive():  # don't overwrite any currently running operations
                t_operation = t_new
                t_operation.start()
            else:
                transmit_msg = "BUSY"
                print("operation currently running... send 'OFF' to kill")
        
        recieved_msg = ''


    except Exception:
        break
    
print("Critical failure. Exiting code!")
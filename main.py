#!/usr/bin/env python3
# pyright: reportMissingImports=false
import os
import time
import argparse
import traceback
import board
from digitalio import DigitalInOut, Direction, Pull  # GPIO module
import meltstake

# parse arguements
argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--mode", help=" mode of operation. Options: debug, deploy", default='deploy')
args = argParser.parse_args()

try:
    meltstake.main(args.mode)
except Exception:
    print(traceback.format_exc())
    time.sleep(10)
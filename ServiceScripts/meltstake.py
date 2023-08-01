#!/usr/bin/env python3
# pyright: reportMissingImports=false
import time
import argparse
import traceback
import sys
sys.path.append('/home/pi/MeltStake')
from .. import main

# parse arguements
argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--mode", help=" mode of operation. Options: debug, deploy", default='deploy')
args = argParser.parse_args()

try:
    main(args.mode)
except Exception:
    print(traceback.format_exc())
    time.sleep(10)
        
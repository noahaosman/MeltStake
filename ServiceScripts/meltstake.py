#!/usr/bin/env python3
# pyright: reportMissingImports=false
from MeltStake import main
import time
import argparse
import traceback

# parse arguements
argParser = argparse.ArgumentParser()
argParser.add_argument("-m", "--mode", help=" mode of operation. Options: debug, deploy", default='deploy')
args = argParser.parse_args()

try:
    main.meltstake(args.mode)
except Exception:
    print(traceback.format_exc())
    time.sleep(10)
        
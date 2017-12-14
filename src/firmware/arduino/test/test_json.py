#! /usr/bin/env python3

import serial
import json
import argparse

parser = argparse.ArgumentParser(description='Read serial json and print to stdout.')
parser.add_argument('device', type=str, help='Device to open')
args = parser.parse_args()

with serial.Serial(args.device, 57600, timeout=1) as ser:
    while True:
        line = ser.readline().decode()
        try:
            j = json.loads(line)
            print(j)
        except json.decoder.JSONDecodeError as e:
            print(e)

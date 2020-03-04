# Code to show that a Raspberry Pi is capable of detecting and selecting specific Arduinos connected to it

import os
import sys
import glob
import serial

print("Scanning USB Ports...")
if sys.platform.startswith("linux"):
    print("Linux detected")
    ports = glob.glob('/dev/tty[USB]*')
else:
    print("Unsupported OS")

result = []

for port in ports:
    try:
        s = serial.Serial(port)
        s.close()
        result.append(port)
    except:
        pass

print(result)

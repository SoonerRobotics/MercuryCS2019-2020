# Code to show that a Raspberry Pi is capable of detecting and selecting specific Arduinos connected to it

import os
import sys
import glob
import serial

print("Scanning USB Ports...")
if sys.platform.startswith("linux"):
    print("Linux detected")
    ports = glob.glob("dev/tty[A-Za-z]*")
else:
    print("Unsupported OS")
    return

result = []

for port in ports:
    try:
        s = serial.Serial(port)
        s.close()
        result.append(port)
    except:
        pass

print(result)

"""
def serial_ports():
    \""" Lists serial port names

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    \"""
    if sys.platform.startswith('win'):
        print("Windows detected")
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        print("Linux detected")
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result
"""

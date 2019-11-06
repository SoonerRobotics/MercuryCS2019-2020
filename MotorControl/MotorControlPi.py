import pygame, sys, serial, json, os, socket
from time import sleep
from enum import Enum
host_ip, server_port = "192.168.1.52", 9999

ser = serial.Serial(timeout = 1) # Set timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB1" # TODO: figure out how to find the port Arduino is connected to automatically
ser.open()

while True:
    tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
       # Establish connection to TCP server and exchange data
        tcp_client.connect((host_ip, server_port))

        # Read data from the TCP server and close the connection
        received = tcp_client.recv(1024)
        ser.write(received)
        dataRead = ser.readline()
        tcp_client.sendall(dataRead)
    finally:
        tcp_client.close()

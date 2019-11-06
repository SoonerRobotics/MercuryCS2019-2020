import pygame, sys, serial, json, os
from time import sleep
from enum import Enum
import socketserver

class Handler_TCPServer(socketserver.BaseRequestHandler):
    """
    The TCP Server class for demonstration.

    Note: We need to implement the Handle method to exchange data
    with TCP client.

    """

    def handle(self):
        # self.request - TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        # Send data to Arduino
        ser.write(self.data)
        # Read data from Arduino
        dataRead = ser.readline()
        # Make sure read data is valid JSON
        try:
            # Print data read from Arduino
            print(json.loads(dataRead.decode("utf-8")))
        except json.decoder.JSONDecodeError:
            print("Error reading from Arduino")
        # Send data from Arduino to client
        self.request.sendall(dataRead)

ser = serial.Serial(timeout = 1) # Set timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB1" # TODO: figure out how to find the port Arduino is connected to automatically
ser.open()

if __name__ == "__main__":
    HOST, PORT = "192.168.1.44", 9999
    socketserver.TCPServer.allow_reuse_address = True
    # Init the TCP server object, bind it to the localhost on 9999 port
    tcp_server = socketserver.TCPServer((HOST, PORT), Handler_TCPServer)

    # Activate the TCP server.
    # To abort the TCP server, press Ctrl-C.
    tcp_server.serve_forever()

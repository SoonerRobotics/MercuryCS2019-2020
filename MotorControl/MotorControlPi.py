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
        print("{} sent:".format(self.client_address[0]))
        print(self.data)
        # Send data to Arduino
        ser.write(json.dumps(dataSend).encode("utf-8"))
        # Read data from Arduino
        dataRead = ser.readline().decode("utf-8")
        # Make sure read data is valid JSON
        try:
            print(json.loads(dataRead))
        except json.decoder.JSONDecodeError:
            print("Error reading from Arduino")
        # just send back ACK for data arrival confirmation
        self.request.sendall("ACK from TCP Server".encode())

ser = serial.Serial(timeout = 1) # Set timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: figure out how to find the port Arduino is connected to automatically
ser.open()

dataSend = {}
dataSend["led"] = True
dataSend["leftStick"] = 0.0
dataSend["rightStick"] = 0.0

if __name__ == "__main__":
    HOST, PORT = "localhost", 9999

    # Init the TCP server object, bind it to the localhost on 9999 port
    tcp_server = socketserver.TCPServer((HOST, PORT), Handler_TCPServer)

    # Activate the TCP server.
    # To abort the TCP server, press Ctrl-C.
    tcp_server.serve_forever()
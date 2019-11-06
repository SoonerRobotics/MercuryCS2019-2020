import pygame, sys, serial, json, os, socketserver
from time import sleep
from enum import Enum

class Handler_TCPServer(socketserver.BaseRequestHandler):
    """
    The TCP Server class for demonstration.

    Note: We need to implement the Handle method to exchange data
    with TCP client.

    """

    def handle(self):
        # self.request - TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        if joystick.get_button(0) == 1:
            dataSend["led"] = True
        elif joystick.get_button(1) == 1:
            dataSend["led"] = False

        # Read y-positions of left and right sticks
        dataSend["leftStick"] = joystick.get_axis(1)
        dataSend["rightStick"] = joystick.get_axis(3)

        # If y-pos of sticks are in "dead zone", round them to 0
        if abs(dataSend["leftStick"]) < DEAD_ZONE_Y:
            dataSend["leftStick"] = 0
        if abs(dataSend["rightStick"]) < DEAD_ZONE_Y:
            dataSend["rightStick"] = 0
        
        self.request.sendall(json.dumps(dataSend).encode())

# Dead zone for y-axis
DEAD_ZONE_Y = 0.1

#setup
pygame.display.init()
pygame.init()
#window = pygame.display.set_mode((200, 200))
#pygame.display.set_caption("Xbox Controller Testing")
joystick = pygame.joystick.Joystick(0)
joystick.init()
dataSend = {}
dataSend["led"] = True
dataSend["leftStick"] = 0.0
dataSend["rightStick"] = 0.0

if __name__ == "__main__":
    HOST, PORT = "192.168.1.52", 9999
    socketserver.TCPServer.allow_reuse_address = True
    # Init the TCP server object, bind it to the localhost on 9999 port
    tcp_server = socketserver.TCPServer((HOST, PORT), Handler_TCPServer)

    # Activate the TCP server.
    # To abort the TCP server, press Ctrl-C.
    tcp_server.serve_forever()
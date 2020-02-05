import pygame, serial, json, socketserver
# Dead zone for y-axis
DEAD_ZONE_Y = 0.1

#setup
pygame.display.init()
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
dataSend = {}

dataSend["led"] = True
dataSend["leftStick"] = 0.0
dataSend["rightStick"] = 0.0


class Handler_UDPServer(socketserver.BaseRequestHandler):
    def handle(self):
        # Receive data from UDP client
        self.data = self.request[0].strip() ##remove leading and ending whitespace
        # Get button/joystick values
        pygame.event.get()
        # Turn on LED if A is pressed   
        if joystick.get_button(0) == 1:
            dataSend["led"] = True
        # Turn off LED if B is pressed
        elif joystick.get_button(1) == 1:
            dataSend["led"] = False
        # Read y-positions of left and right sticks
        dataSend["leftStick"] = joystick.get_axis(1)
        dataSend["rightStick"] = joystick.get_axis(3)

        # If y-pos of sticks are in "dead zone", round them to 0
        # TODO: pygame can do this for us
        if abs(dataSend["leftStick"]) < DEAD_ZONE_Y:
            dataSend["leftStick"] = 0
        if abs(dataSend["rightStick"]) < DEAD_ZONE_Y:
            dataSend["rightStick"] = 0

        # Send JSON to pi
        self.request[1].sendto(json.dumps(dataSend).encode(), self.client_address)

        
if __name__ == "__main__":
    # TODO: figure out a better way to find the host ip
    HOST, PORT = "192.168.1.52", 9999
    # TODO: do we still need this? leftover from switch from TCP to UDP
    socketserver.UDPServer.allow_reuse_address = True
    # Init the UDP server object, bind it to the localhost on 9999 port
    udp_server = socketserver.UDPServer((HOST, PORT), Handler_UDPServer)

    # Activate the UDP server.
    # To abort the UDP server, press Ctrl-C.
    udp_server.serve_forever()

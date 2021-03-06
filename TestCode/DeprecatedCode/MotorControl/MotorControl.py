import pygame, sys, serial, json, os
from time import sleep
from enum import Enum

# Dead zone for y-axis
DEAD_ZONE_Y = 0.1

# Because there is only 1 hat, it doesn't need an enum. Returns a coordinate (x,y)
#Enum Class describing all of the Buttons
class XboxButtons(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    LJOY = 8
    RJOY = 9

#Enum Class describing all of the axes
class XboxAxes(Enum):
    LY = 0
    LX = 1
    T = 2 #Analog triggers, right side negative - result is sum of them (-1, 1)
    RX = 3
    RY = 4

#setup
pygame.display.init()
pygame.init()
#window = pygame.display.set_mode((200, 200))
#pygame.display.set_caption("Xbox Controller Testing")
joystick = pygame.joystick.Joystick(0)
joystick.init()

ser = serial.Serial(timeout = 1) # Set timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: figure out how to find the port Arduino is connected to automatically
ser.open()

dataSend = {}
dataSend["led"] = True
dataSend["leftStick"] = 0.0
dataSend["rightStick"] = 0.0
done = False
while (not done):
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
    ''' For debugging: prints pressed button(s)
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")
    window.fill(pygame.Color('white'))
    pressedButtons = []
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i) == 1:
            pressedButtons.append(XboxButtons(i))
    for n in pressedButtons:
        print(n.name + " ")
    '''
    if joystick.get_button(0) == 1:
        dataSend["led"] = True
    if joystick.get_button(1) == 1:
        dataSend["led"] = False

    # Read y-positions of left and right sticks
    dataSend["leftStick"] = joystick.get_axis(1)
    dataSend["rightStick"] = joystick.get_axis(4)

    # If y-pos of sticks are in "dead zone", round them to 0
    if abs(dataSend["leftStick"]) < DEAD_ZONE_Y:
        dataSend["leftStick"] = 0
    if abs(dataSend["rightStick"]) < DEAD_ZONE_Y:
        dataSend["rightStick"] = 0

    # Send data to Arduino
    ser.write(json.dumps(dataSend).encode("utf-8"))
    # Read data from Arduino
    dataRead = ser.readline().decode("utf-8")
    # Make sure read data is valid JSON
    try:
        print(json.loads(dataRead))
    except json.decoder.JSONDecodeError:
        print("Error reading from Arduino")

pygame.quit()

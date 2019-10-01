import pygame, sys, serial, json, time
from time import sleep
from enum import Enum


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
    LX = 0
    LY = 1
    T = 2 #Analog triggers, right side negative - result is sum of them (-1, 1)
    RX = 3
    RY = 4

#setup
pygame.init()
window = pygame.display.set_mode((200, 200))
pygame.display.set_caption("Xbox Controller Testing")
joystick = pygame.joystick.Joystick(0)
joystick.init()

ser = serial.Serial()
ser.baudrate = 38400
ser.port = "COM9"
ser.open()

dataSend = {}
dataSend["led"] = True
dataSend["leftStick"] = 0.0 
time.sleep(0.5)
done = False
while (not done):
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
    window.fill(pygame.Color('white'))
    pressedButtons = []
    for i in range(joystick.get_numbuttons()):
        if joystick.get_button(i) == 1:
            pressedButtons.append(XboxButtons(i))
    for n in pressedButtons:
        print(n.name + " ")

    if joystick.get_button(0) == 1:
        dataSend["led"] = True
    if joystick.get_button(1) == 1:
        dataSend["led"] = False
    dataSend["leftStick"] = joystick.get_axis(1)
    if abs(dataSend["leftStick"]) < 0.1:
        dataSend["leftStick"] = 0

    # Send data to Arduino
    ser.write(json.dumps(dataSend).encode("utf-8"))
    dataRead = ser.readline().decode("utf-8")
    print(json.loads(dataRead))

pygame.quit()
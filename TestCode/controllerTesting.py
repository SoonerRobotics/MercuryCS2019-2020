import pygame, sys
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

done = False
while (not done):
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
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

pygame.quit()
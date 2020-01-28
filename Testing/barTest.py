import sys, pygame, time, serial, json

# Set up Serial
ser = serial.Serial()
ser.baudrate = 38400
ser.port = "COM9"
ser.open()

pygame.init()

# Set up a wide screen so our bar can fit
screenSize = width, height = 1200, 240
screen = pygame.display.set_mode(screenSize)

# text setting
font_obj = pygame.font.Font('freesansbold.ttf', 32)
text_surface_obj = font_obj.render("0", True, (0,255,0), (0,0,180))
text_rect_obj = text_surface_obj.get_rect()
text_rect_obj.center = (200, 150)

while True:
    # Check for invalid JSON from the Arduino so the program doesn't crash
    try:
        # Read the JSON from the Arduino
        data = json.loads(ser.readline().decode("utf-8"))
        print(data)
        # Draw a blank white screen
        screen.fill((255,255,255))
        # Add text displaying the sensor value
        text_surface_obj = font_obj.render("%d"%int(data["sensor"]), True, (0,255,0), (0,0,180))
        # Add a rectangle with width equal to the sensor value
        pygame.draw.rect(screen, (0,0,0), (64,64,int(data["sensor"]),64), 0)
        # Draw everything on the screen
        screen.blit(text_surface_obj, text_rect_obj)
        pygame.display.flip()
    # Did not receive valid JSON, ignore it
    except ValueError as error:
        pass
    # The Arduino is sending data faster than we are reading it, so we need to clear the input buffer to get rid of outdated data
    ser.flushInput()
    time.sleep(0.05)
import sys, serial, json, socket, cv2
host_ip, server_port = "192.168.1.52", 9999

ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
ser.open()

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

vc = cv2.VideoCapture(0) #Set this to 1 in order to use the webcamera if you have one built in

if vc.isOpened():
    rval, frame = vc.read()
else:
    rval = False

data = "Client start. Camera: " + str(rval)

dataRead = data.encode()

#Retrieving the properties for the frame size
width = vc.get(3)   # float
height = vc.get(4) # float

width = int(width)
height = int(height)

oldDimensions = (width, height) #For upscaling after downscaling, so that the size of the image appears constant

scale_percent = 25 #Modify this to change the quality of the image, 100 equals no change
width = width * scale_percent / 100
height = height * scale_percent / 100

width = int(width)
height = int(height)

newDimensions = (width, height)

#Changing image quality
downscale = cv2.resize(frame, newDimensions, interpolation = cv2.INTER_AREA)
resized = cv2.resize(downscale, oldDimensions, interpolation = cv2.INTER_AREA)

#Changing image color
gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

img_str = cv2.imencode('.jpg', gray)[1].tostring()

udp_client.sendto(dataRead, (host_ip, server_port))
# udp_client.sendto(img_str, (host_ip, server_port))
print(sys.getsizeof(img_str))
while True:
    # Try loop to prevent horrible issues should errors get thrown
    try:
        # Send data to UDP Server
        # udp_client.sendto(dataRead, (host_ip, server_port))
        # Read data from the UDP server
        received = udp_client.recv(1024)
        # Print data received from server
        print(received.decode())
        # Send data read from server to Arduino
        ser.write(received)
        # Read from Arduino
        dataRead = ser.readline()
    finally:
        pass

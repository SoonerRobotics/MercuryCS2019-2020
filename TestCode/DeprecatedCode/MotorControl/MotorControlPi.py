import sys, serial, json, socket, datetime, time
host_ip, server_port = "192.168.1.52", 9999

ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
ser.open()

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

dataRead = "".encode()

# Main control loop
while True:
    try:
        # Send data to UDP Server
        udp_client.sendto(dataRead, (host_ip, server_port))
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

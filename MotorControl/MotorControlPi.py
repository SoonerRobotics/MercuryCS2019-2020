import sys, serial, json, socket
host_ip, server_port = "192.168.1.52", 9999

ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
ser.open()

dataRead = "Client start".encode()

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
while True:
    # TODO: do we need to use try?
    try:
        # Send data to UDP Server
        udp_client.sendto(dataRead, (host_ip, server_port))
        # Read data from the UDP server
        received = udp_client.recv(1024)
        # Send data read from server to Arduino
        ser.write(received)
        # Print data received from server
        print(received.decode())
        # Read from Arduino
        dataRead = ser.readline()
    finally:
        pass

#! /usr/bin/env python

import socket
import json

import rospy
import serial

from mercury_msgs.msg import sensors

# Define serial connection
ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
ser.baudrate = 38400
ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
ser.connected = False
while not ser.connected:
    try:
        ser.open()
    except serial.SerialException as e:
        print(f"Error: {e}")
    else:
        ser.connected = True
        time.sleep(.5)
        
def receive_sensors():
    pub = rospy.Publisher("/mercury/sensors", sensors, queue_size=1)
    # Initialize ROS node
    rospy.init_node("serial_sensor_receiver_node")
    sensor_values = sensors()
    while not rospy.is_shutdown():
        # Read data from the UDP server
        dataReceived = ser.readline()
        try:
            decodedData = dataReceived.decode()
            data = json.loads(decodedData)
        except:
            pass
        sensor_values.us_front = data["us_front"]
        sensor_values.us_left = data["us_left"]
        sensor_values.us_right = data["us_right"]
        sensor_values.mag_x = data["mag_x"]
        sensor_values.mag_y = data["mag_y"]
        sensor_values.mag_z = data["mag_z"]
        pub.publish(sensor_values)

if __name__ == "__main__":
    try:
        receive_sensors()
    except rospy.ROSInterruptException:
        pass

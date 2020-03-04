#! /usr/bin/env python

import socket
import json
import rospy
import serial
import time

from mercury_msgs.msg import sensors

def receive_sensors():
    pub = rospy.Publisher("/mercury/sensors", sensors, queue_size=1)
    # Initialize ROS node
    rospy.init_node("serial_sensor_receiver_node")
    sensor_values = sensors()
    while not rospy.is_shutdown():
        dataReceived = ser.readline()
        try:
            decodedData = dataReceived.decode()
            data = json.loads(decodedData)
            sensor_values.us_front = data["us_front"]
            sensor_values.us_left = data["us_left"]
            sensor_values.us_right = data["us_right"]
            sensor_values.mag_x = data["mag_x"]
            sensor_values.mag_y = data["mag_y"]
            sensor_values.mag_z = data["mag_z"]
	    rospy.loginfo(sensor_values)
            pub.publish(sensor_values)
	except:
	    pass

if __name__ == "__main__":
    # Define serial connection
    ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
    ser.baudrate = 38400
    ser.port = "/dev/ttyUSB0"
    ser.connected = False
    while not ser.connected:
        try:
            ser.open()
        except serial.SerialException as e:
            rospy.loginfo("Error: {e}")
        else:
            ser.connected = True
            time.sleep(.5)

    try:
	data = json.loads(ser.readline().decode())
    except:
	pass
    else:
	if data["sensors"] == 0:
	    ser.close()
	    ser.port = "/dev/ttyUSB1"
	    ser.connected = False
	    while not ser.connected:
		try:

    try:
        receive_sensors()
    except rospy.ROSInterruptException:
        pass

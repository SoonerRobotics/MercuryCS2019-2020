#! /usr/bin/env python

from mercury_msgs.msg import sensors
import json
import rospy
import serial
import time

def receive_serial():
    pub = rospy.Publisher("/mercury/sensors", sensors, queue_size=1)
    sensor_values = sensors()
    while not rospy.is_shutdown():
        try:
	    # Read and decode Serial data from Arduinos
            data_USB0 = json.loads(ser_USB0.readline().decode())
            data_USB1 = json.loads(ser_USB1.readline().decode())
        except Exception as e:
            rospy.loginfo("Error: " + str(e))
        else:
	    # Arduino connected to USB0 is the sensor Arduino
            if data_USB0["sensor"] == 1:
                sensor_values.us_front = data_USB0["us_front"]
                sensor_values.us_left = data_USB0["us_left"]
                sensor_values.us_right = data_USB0["us_right"]
                sensor_values.mag_x = data_USB0["mag_x"]
                sensor_values.mag_y = data_USB0["mag_y"]
                sensor_values.mag_z = data_USB0["mag_z"]
		sensor_values.enc_left = data_USB1["enc_left"]
		sensor_values.enc_right = data_USB1["enc_right"]
		sensor_values.heading = data_USB1["heading"]
	    # Arduino connected to USB1 is the sensor Arduino
            else:
                sensor_values.us_front = data_USB1["us_front"]
                sensor_values.us_left = data_USB1["us_left"]
                sensor_values.us_right = data_USB1["us_right"]
                sensor_values.mag_x = data_USB1["mag_x"]
                sensor_values.mag_y = data_USB1["mag_y"]
                sensor_values.mag_z = data_USB1["mag_z"]
		sensor_values.enc_left = data_USB0["enc_left"]
		sensor_values.enc_right = data_USB0["enc_right"]
		sensor_values.heading = data_USB0["heading"]
	    #rospy.loginfo(sensor_values)
	    pub.publish(sensor_values)

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("serial_receiver_node")
    # Define serial connection for USB0
    ser_USB0 = serial.Serial(timeout = 1) # Set serial timeout to 1 second
    ser_USB0.baudrate = 38400
    ser_USB0.port = "/dev/ttyUSB0"
    ser_USB0.connected = False
    while not ser_USB0.connected:
        try:
            ser_USB0.open()
        except Exception as e:
            rospy.loginfo("Error: " + str(e))
        else:
            ser_USB0.connected = True
            time.sleep(0.5)

    # Define serial connection for USB1
    ser_USB1 = serial.Serial(timeout = 1) # Set serial timeout to 1 second
    ser_USB1.baudrate = 38400
    ser_USB1.port = "/dev/ttyUSB1"
    ser_USB1.connected = False
    while not ser_USB1.connected:
        try:
            ser_USB1.open()
        except Exception as e:
            rospy.loginfo("Error: " + str(e))
        else:
            ser_USB1.connected = True
            time.sleep(0.5)

    try:
        receive_serial()
    except rospy.ROSInterruptException:
        pass

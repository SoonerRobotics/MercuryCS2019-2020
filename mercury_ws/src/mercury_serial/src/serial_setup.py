#! /usr/bin/env python

from mercury_msgs.msg import ports
import json
import rospy
import serial
import time

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("serial_setup")
    pub = rospy.Publisher("/mercury/ports", ports, queue_size=1)
    port_names = ports()
    ser_USB0 = serial.Serial(timeout = 1)
    ser_USB0.baudrate = 38400
    ser_USB0.port = "/dev/ttyUSB0"
    ser_USB0.connected = False
    while not ser_USB0.connected:
	try:
	    ser_USB0.open()
	except Exception as e:
	    rospy.loginfo_throttle(1, "Error: {}".format(str(e)))
	else:
	    ser_USB0.connected = True
	    time.sleep(0.5)

    ser_USB1 = serial.Serial(timeout = 1)
    ser_USB1.baudrate = 38400
    ser_USB1.port = "/dev/ttyUSB1"
    ser_USB1.connected = False

    while not ser_USB1.connected:
        try:
            ser_USB1.open()
        except Exception as e:
            rospy.loginfo_throttle(1, "Error: {}".format(str(e)))
        else:
            ser_USB1.connected = True
            time.sleep(0.5)


    ser_USB0.read_done = False
    while not ser_USB0.read_done:
	try:
	    data_USB0 = json.loads(ser_USB0.readline().decode())
	except Exception as e:
	    rospy.loginfo_throttle(1, "Error reading from {}: {}".format(ser_USB0.port, str(e)))
	else:
	    ser_USB0.close()
	    ser_USB0.read_done = True

    ser_USB1.read_done = False
    while not ser_USB1.read_done:
        try:
            data_USB1 = json.loads(ser_USB1.readline().decode())
        except Exception as e:
            rospy.loginfo_throttle(1, "Error reading from {}: {}".format(ser_USB1.port, str(e)))
        else:
	    ser_USB1.close()
            ser_USB1.read_done = True

    rospy.loginfo("Successfully read JSON from both Arduinos")

    if (data_USB0["sensor"] == 0) and (data_USB1["sensor"] == 1):
	port_names.motor_port = ser_USB0.port
	port_names.sensor_port = ser_USB1.port
    elif (data_USB0["sensor"] == 1) and (data_USB1["sensor"] == 0):
	port_names.sensor_port = ser_USB0.port
        port_names.motor_port = ser_USB1.port
    else:
	rospy.loginfo("Connected devices are not set up properly")

    rospy.loginfo("Arduino ports: %s", port_names)
    pub.publish(port_names)

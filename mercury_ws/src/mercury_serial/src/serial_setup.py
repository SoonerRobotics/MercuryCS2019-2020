#! /usr/bin/env python

from mercury_msgs.msg import ports
import json
import rospy
import serial
import time
import glob

if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node("serial_setup")
    pub = rospy.Publisher("/mercury/ports", ports, queue_size=1)
    # Get USB ports that are connected to an Arduino
    con_ports = glob.glob('/dev/tty[USB]*')
    # Make sure only two USB ports have an Arduino connected
    if len(con_ports) is not 2:
        raise SystemExit("Error: make sure exactly 2 Arduinos are connected")
    port_names = ports()
    # TODO: check ttyUSB0 to ttyUSB4 instead of just ttyUSB0 and ttyUSB1
    ser_A = serial.Serial(timeout = 1)
    ser_A.baudrate = 38400
    ser_A.port = con_ports[0]
    ser_A.connected = False
    while not ser_A.connected:
	    try:
	        ser_A.open()
	    except Exception as e:
	        rospy.loginfo_throttle(1, "Error: {}".format(str(e)))
	    else:
	        ser_A.connected = True
	        time.sleep(0.5)

    ser_B = serial.Serial(timeout = 1)
    ser_B.baudrate = 38400
    ser_B.port = con_ports[1]
    ser_B.connected = False
    while not ser_B.connected:
        try:
            ser_B.open()
        except Exception as e:
            rospy.loginfo_throttle(1, "Error: {}".format(str(e)))
        else:
            ser_B.connected = True
            time.sleep(0.5)

    ser_A.read_done = False
    while not ser_A.read_done:
	    try:
	        data_A = json.loads(ser_A.readline().decode())
	    except Exception as e:
	        rospy.loginfo_throttle(1, "Error reading from {}: {}".format(ser_A.port, str(e)))
	    else:
	        ser_A.close()
	        ser_A.read_done = True

    ser_B.read_done = False
    while not ser_B.read_done:
        try:
            data_B = json.loads(ser_B.readline().decode())
        except Exception as e:
            rospy.loginfo_throttle(1, "Error reading from {}: {}".format(ser_B.port, str(e)))
        else:
            ser_B.close()
            ser_B.read_done = True

    rospy.loginfo("Successfully read JSON from both Arduinos")

    if (data_A["sensor"] == 0) and (data_B["sensor"] == 1):
	    port_names.motor_port = ser_A.port
	    port_names.sensor_port = ser_B.port
    elif (data_A["sensor"] == 1) and (data_B["sensor"] == 0):
        port_names.sensor_port = ser_A.port
        port_names.motor_port = ser_B.port
    else:
	    rospy.loginfo("Connected devices are not set up properly")

    rospy.loginfo("Arduino ports: %s", port_names)
    pub.publish(port_names)

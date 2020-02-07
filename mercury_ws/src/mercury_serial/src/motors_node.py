#! /usr/bin/env python

import rospy
import serial
import json
from mercury_msgs.msg import motors
from std_msgs.msg import String

def write_motors(data):
    # rospy.loginfo(data)
    dataDict = {
        "leftMotor" : data.left,
        "rightMotor" : data.right
    }
    ser.write(json.dumps(dataDict).encode())

def read_sensors(event):
    pub = rospy.Publisher("/mercury/motor_sensors", String, queue_size=1)
    # Publish sensor data sent from motor Arduino
    pub.publish(ser.read_until('\n'))

def run_motors():
    rospy.init_node("mercury_motors_node")
    rospy.loginfo("Motor Node Started")
    rospy.Subscriber("/mercury/motor_out", motors, write_motors)

    # Read sensors 10 times/sec
    rospy.Timer(rospy.Duration(0.1), read_sensors)

    rospy.spin()

if __name__ == "__main__":
    ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
    ser.baudrate = 38400 # TODO: increase baudrate?
    ser.port = "/dev/ttyUSB0" # TODO: use something to find the port the Arduino is connected to automatically
    ser.open()
    try:
        run_motors()
    except rospy.ROSInterruptException:
        pass
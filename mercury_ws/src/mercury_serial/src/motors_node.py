#! /usr/bin/env python

import rospy
import serial
import json
from mercury_msgs.msg import motors
from mercury_msgs.msg import ports
from mercury_msgs.msg import motor_sensors

def wait_for_port():
    rospy.init_node("motors_node")
    rospy.Subscriber("mercury/ports", ports, run_motors)
    rospy.spin()

def write_motors(data):
    # rospy.loginfo(data)
    dataDict = {
        "leftMotor" : data.left,
        "rightMotor" : data.right
    }
    ser.write(json.dumps(dataDict).encode())

def read_sensors(event):
    pub = rospy.Publisher("/mercury/motor_sensors", motor_sensors, queue_size=1)
    sensor_values = motor_sensors()
    try:
        sensor_data = json.loads(ser.readline().decode())
    except Exception as e:
        rospy.loginfo("Error reading motor sensor data: %s", str(e))
    else:
        # If read was successful, publish motor Arduino's sensor data
        sensor_values.enc_left = sensor_data["enc_left"]
        sensor_values.enc_right = sensor_data["enc_right"]
        sensor_values.heading = sensor_data["heading"]
        #rospy.loginfo(sensor_values)
        pub.publish(sensor_values)

def run_motors(port_names):
    ser.baudrate = 38400
    ser.port = port_names.motor_port
    ser.connected = False
    # Connect to motor Arduino serial port
    while not ser.connected:
        try:
            ser.open()
        except Exception as e:
            rospy.loginfo_throttle(1, str(e))
        else:
            ser.connected = True

    rospy.Subscriber("/mercury/motor_out", motors, write_motors)

    # Read sensors 10 times/sec
    rospy.Timer(rospy.Duration(0.1), read_sensors)

    rospy.spin()

if __name__ == "__main__":
    ser = serial.Serial(timeout = 1)
    try:
        wait_for_port()
    except rospy.ROSInterruptException:
        pass

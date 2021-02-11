#! /usr/bin/env python

from mercury_msgs.msg import sensors
from mercury_msgs.msg import ports
import json
import rospy
import serial

def wait_for_port():
    rospy.init_node("serial_sensor_receiver")
    rospy.Subscriber("mercury/ports", ports, receive_sensors)
    rospy.spin()

def receive_sensors(port_names):
    pub = rospy.Publisher("/mercury/sensors", sensors, queue_size=1)
    sensor_values = sensors()
    ser = serial.Serial(timeout = 1)
    ser.baudrate = 38400
    ser.port = port_names.sensor_port
    ser.connected = False
    # Connect to sensor Arduino serial port
    while not ser.connected:
        try:
            ser.open()
        except Exception as e:
            rospy.loginfo_throttle(1, str(e))
        else:
            ser.connected = True

    while not rospy.is_shutdown():
        try:
            sensor_data = json.loads(ser.readline().decode())
        except Exception as e:
            rospy.loginfo("Error reading sensor data: %s", str(e))
        else:
            # If read was successful, publish sensor data
            sensor_values.us_front = sensor_data["us_front"]
            sensor_values.us_left = sensor_data["us_left"]
            sensor_values.us_right = sensor_data["us_right"]
            sensor_values.mag_x = sensor_data["mag_x"]
            sensor_values.mag_y = sensor_data["mag_y"]
            sensor_values.mag_z = sensor_data["mag_z"]
            #rospy.loginfo(sensor_values)
            pub.publish(sensor_values)

if __name__ == "__main__":
    try:
        wait_for_port()
    except rospy.ROSInterruptException:
        pass

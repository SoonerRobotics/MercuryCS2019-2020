#! /usr/bin/env python

import rospy
import socket
import json
from mercury_msgs.msg import motors

host_ip = "10.8.0.2"
server_port = 9999

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def receive_udp():
    pub = rospy.Publisher("/mercury/motors", motors, queue_size=1)
    # Initialize ROS node
    rospy.init_node("mercury_udp_node")
    motor_values = motors()
    while not rospy.is_shutdown():
        # Read data from the UDP server
        dataReceived = udp_client.recv(1024)
        try:
            decodedData = dataRecevied.decode()
            data = json.loads(decodedData)
        except:
            pass
        motor_values.left = data["leftStick"]
        motor_values.right = data["rightStick"]
        pub.publish(motor_values)

if __name__ == "__main__":
    try:
        receive_udp()
    except rospy.ROSInterruptException:
        pass
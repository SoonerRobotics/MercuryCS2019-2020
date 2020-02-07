#! /usr/bin/env python

import rospy
import socket
import json
from mercury_msgs.msg import motors

# This depends on the IP of the device running the UDP server (the driver's device)
host_ip = "10.8.0.4"
server_port = 9999

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def receive_udp():
    # Initialize ROS node
    rospy.init_node("mercury_udp_node")
    rospy.loginfo("UDP Node Started")
    pub = rospy.Publisher("/mercury/motor_in", motors, queue_size=1)
    motor_values = motors()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # TODO: Send meaningful data back here
        udp_client.sendto("", (host_ip, server_port))
        # Read data from the UDP server
        dataReceived = udp_client.recv(1024)
        # rospy.loginfo(dataReceived)
        try:
            decodedData = dataReceived.decode()
            data = json.loads(decodedData)
            motor_values.left = data["leftStick"]
            motor_values.right = data["rightStick"]
            pub.publish(motor_values)
        except:
            pass
        rate.sleep()
    #rospy.spin()

if __name__ == "__main__":
    try:
        receive_udp()
    except rospy.ROSInterruptException:
        pass
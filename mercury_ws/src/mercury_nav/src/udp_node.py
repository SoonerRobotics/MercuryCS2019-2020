#! /usr/bin/env python

import rospy
import socket
import json
from mercury_msgs.msg import motors

# This depends on the IP of the device running the UDP server (the driver's device)
host_ip = "10.8.0.2"
server_port = 9999

udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_client.settimeout(1) # Timeout after 1 second

def receive_udp():
    # Initialize ROS node
    rospy.init_node("mercury_udp_node")
    rospy.loginfo("UDP Node Started")
    pub = rospy.Publisher("/mercury/motor_in", motors, queue_size=1)
    motor_values = motors()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            # TODO: Send meaningful data back here
            udp_client.sendto("", (host_ip, server_port))
            # Read data from the UDP server
            dataReceived = udp_client.recv(1024)
            # rospy.loginfo(dataReceived)
        except Exception as e:
            rospy.loginfo("Error sending/receiving over UDP: %s", str(e))
            # Error getting control data, so stop the motors
            motor_values.left = 0
            motor_values.right = 0
        else:
            try:
                data = json.loads(dataReceived.decode())
            except Exception as e:
                rospy.loginfo(str(e))
            else:
                motor_values.left = data["leftStick"]
                motor_values.right = data["rightStick"]

        rospy.loginfo(motor_values)
        pub.publish(motor_values)
        rate.sleep()
    #rospy.spin()

if __name__ == "__main__":
    try:
        receive_udp()
    except rospy.ROSInterruptException:
        pass

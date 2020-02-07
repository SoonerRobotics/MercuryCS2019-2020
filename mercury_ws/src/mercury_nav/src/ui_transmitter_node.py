#! /usr/bin/env python

import socket
import json
import datetime
import time

import rospy
import cv2
import imutils
from imutils.video import VideoStream

from mercury_msgs.msg import sensors

# Sensor values updated by callback
sensor_values = sensors()

# Represents pi video stream
vs = VideoStream(src=0).start()

def send_tcp():

    # How often this function repeats in hertz
    hz = 15

    # The IP address of the UI server and the port that the UI receive is running
    host_ip = "192.168.1.74"
    server_port = 9000 # Don't change this value
    
    rospy.init_node("ui_transmitter_node")
    rospy.loginfo("UI Transmitter Node Started")
    rospy.Subscriber("serial_sensor_receiver_node", sensors, sensor_update_callback)

    rospy.loginfo("Trying to connect to server {}:{}".format(host_ip, server_port))
    while True:
        try:
            tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_client.connect((host_ip, server_port))
            c_status = "Connected"
            rospy.loginfo(c_status)
        except Exception as e:
            rospy.loginfo("Error: {}".format(e))

            # While connected, write sensors to server
        while c_status == "Connected":

            send_dict = get_send_dict()

            send_msg = json.dumps(send_dict) + "\n"
            send_msg = send_msg.encode()

            try:
                tcp_client.sendall(send_msg)
            except Exception as e:
                c_status = "Disconnected"
                rospy.loginfo("Error: {}".format(e))
        
        time.sleep(1/hz)

def get_send_dict():

    global sensor_values

    sensor_dict = {}
    sensor_dict["frame"] = fetch_frame().tolist()
    sensor_dict["us_front"] = sensor_values.us_front
    sensor_dict["us_left"] = sensor_values.us_left
    sensor_dict["us_right"] = sensor_values.us_right
    sensor_dict["mag_x"] = sensor_values.mag_x
    sensor_dict["mag_y"] = sensor_values.mag_y
    sensor_dict["mag_z"] = sensor_values.mag_z

    return sensor_dict

def send_dict(sensor_dict):
    pass
        
def fetch_frame():

    global vs
    
    # Grab a frame from the camera stream
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    # Grab the current timestamp and draw it on the frame
    timestamp = datetime.datetime.now()
    cv2.putText(frame, timestamp.strftime(
        "%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    # Encode raw frame as a jpeg
    flag, encodedImage = cv2.imencode(".jpg", frame)

    return encodedImage

def sensor_update_callback(data):

    global sensor_values

    sensor_values = data

    
        
if __name__ == "__main__":
    try:
        send_tcp()
    except rospy.ROSInterruptException:
        pass

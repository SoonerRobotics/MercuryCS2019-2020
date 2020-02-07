#! /usr/bin/env python

import socket
import json
import datetime
import time

import rospy
import imutils
from imutils.video import VideoStream

from mercury_msgs.msg import sensors

# This depends on the IP of the device running the UDP server (the driver's device)
host_ip = "192.168.1.74" # IP of laptop using UI software
server_port = 9000

# How often to send data to UI
hz = 15

# Sensor values updated by callback
sensor_values = sensors()

# Represents pi video stream
vs = VideoStream(src=0).start()

def send_tcp():

    global sensor_values
    
    rospy.init_node("ui_transmitter_node")
    rospy.loginfo("UI Transmitter Node Started")
    rospy.Subscriber("serial_sensor_receiver_node", sensors, sensor_update_callback)
    
    while not rospy.is_shutdown():

        sensor_dict = {}
        sensor_dict["frame"] = fetch_frame()
        sensor_dict["us_front"] = sensor_values.us_front
        sensor_dict["us_left"] = sensor_values.us_left
        sensor_dict["us_right"] = sensor_values.us_right
        sensor_dict["mag_x"] = sensor_values.mag_x
        sensor_dict["mag_y"] = sensor_values.mag_y
        sensor_dict["mag_z"] = sensor_values.mag_z
        
        time.sleep(1/hz)

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

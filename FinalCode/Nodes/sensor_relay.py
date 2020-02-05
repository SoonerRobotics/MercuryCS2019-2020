#!/usr/bin/env python3

# Pipeline:
#   Input from arduino sensors (blocking) and pi camera
#   Output to UI (on udp client)
#   Output to nav (on queue)
#   Output to obstacle avoidance (on queue)

import sys, serial, json, socket, datetime, time, threading, copy, math, sys

import numpy as np
import cv2
import imutils
from imutils.video import VideoStream

import help_lib as hl

def main(write_queue=None, picam=False, local=False):
    """Read sensors from Arduino and pipe to other processes"""

    global data_dict, lock, logger, c_status

    # Create logger
    logger = hl.create_logger(__name__)

    # Initial status
    c_status = "Disconnected"

    # Dummy numpy black frame
    dummy_flag, dummy_frame = cv2.imencode(".jpg", np.zeros(shape=(250, 400, 3), dtype=np.uint8))

    # Define data dict to be send
    data_dict = {
        "frame": dummy_frame,
        "us_front": 1,
        "us_left": 1,
        "us_right": 1,
        "a": 1,
        "m": 1
    }

    # Define thread lock
    lock = threading.Lock()

    def fetch_frame(vs):
        """Fetch a frame from the video camera"""

        # Grab a frame from the camera stream
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # grab the current timestamp and draw it on the frame
        timestamp = datetime.datetime.now()
        cv2.putText(frame, timestamp.strftime(
            "%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        # Encode raw frame as a jpeg
        flag, encodedImage = cv2.imencode(".jpg", frame)

        return encodedImage

    def sensor_read():
        """Read the sensors from the Arduino"""
        global data_dict, lock, logger

        # Define serial connection
        ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
        ser.baudrate = 38400
        ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
        ser.connected = False

        # Attempt to connect to Arduino until successful
        while not ser.connected:
            try:
                ser.open()
            except serial.SerialException as e:
                logger.warn(f"Error: {e}")
            else:
                ser.connected = True
            time.sleep(.5)

        # Read sensors for forever
        while True:

            received = json.loads(ser.readline().decode())
            with lock:
                # Write received data dict to global variable with lock
                data_dict = copy.deepcopy(received)

    def cam_read(picam=True):
        """Read the camera"""
        global data_dict, lock, logger

        # Start up camera
        camera_active = False
        while not camera_active:
            try:
                if picam:
                    vs = VideoStream(usePiCamera=1).start()
                    camera_active = True
                else:
                    vs = VideoStream(src=0).start()
                    camera_active = True
                time.sleep(2.0)
            except Exception as e:
                logger.warn(e)


            # Fetch frames for forever
            while camera_active:
                try:
                    frame = fetch_frame(vs)
                    with lock:
                        # Write frame to dict, since it's only available to the pi
                        data_dict["frame"] = copy.deepcopy(frame)
                except:
                    camera_active = False


    def queue_write(queue=None):
        """Write to multiprocess output queue"""
        global data_dict, lock

        while True:
            # Write data dict to queues
            # Put data dictionary into queue if queue isn't full
            if queue is not None:
                with lock:
                    hl.myput(queue, data_dict)
                            
            # Break if no queues
            else:
                break

    def server_write(local=False):
        """Write to ui server"""
        global data_dict, lock, logger, c_status

        # Define server ip, port, and client
        server_port = 9000
        if local:
            host_ip = "localhost"
        else:
            host_ip = "192.168.1.74"

        # Attempt to connect to ui server forever
        logger.info(f"Client Target Address: {host_ip}:{server_port}")
        while True:

            try:
                tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp_client.connect((host_ip, server_port))
                c_status = "Connected"
                logger.info(c_status)
            except Exception as e:
                logger.warn(f"Error: {e}")

            # While connected, write sensors to server
            while c_status == "Connected":

                with lock:
                    send_dict = copy.deepcopy(data_dict)

                if send_dict["frame"] is not None:
                    send_dict["frame"] = send_dict["frame"].tolist()
                send_msg = json.dumps(send_dict) + "\n"
                send_msg = send_msg.encode()

                try:
                    tcp_client.sendall(send_msg)
                except Exception as e:
                    c_status = "Disconnected"
                    logger.warn(e)

    # Spin up threads
    threads = []
    threads.append(threading.Thread(target=sensor_read))
    threads.append(threading.Thread(target=cam_read, args=(picam,)))
    threads.append(threading.Thread(target=queue_write, args=(write_queue,)))
    threads.append(threading.Thread(target=server_write, args=(local,)))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()


if __name__ == "__main__":
    main()
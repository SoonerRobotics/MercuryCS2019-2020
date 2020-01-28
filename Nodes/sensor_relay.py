#!/usr/bin/env python3

# Pipeline:
#   Input from arduino sensors (blocking) and pi camera
#   Output to UI (on udp client)
#   Output to nav (on queue)
#   Output to obstacle avoidance (on queue)

import sys, serial, json, socket, datetime, time, threading, copy, math

import numpy as np
import cv2
from imutils.video import VideoStream

def main(write_queue=None, picam=True, local_server=False):
    global data_dict, lock

    dummy_flag, dummy_frame = cv2.imencode(".jpg", np.zeros(shape=(250, 400, 3), dtype=np.uint8))

    # Define data dict to be send
    data_dict = {
        "frame": None,
        "us_front": 1,
        "us_left": 1,
        "us_right": 1,
        "a": 1,
        "m": 1
    }

    # Define thread lock
    lock = threading.Lock()

    def fetch_frame(vs):

        # Grab a frame from the camera stream
        frame = vs.read()
        frame = imutils.resize(frame, width=400)

        # grab the current timestamp and draw it on the frame
        timestamp = datetime.datetime.now()
        cv2.putText(frame, timestamp.strftime(
            "%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

        #flag, encodedImage = cv2.imencode(".jpg", frame)

        return frame

    def sensor_read(picam=True):
        global data_dict, lock

        # Start up camera
        if picam:
            vs = VideoStream(usePiCamera=1).start()
        else:
            vs = VideoStream(src=0).start()
        time.sleep(2.0)

        # Define serial connection
        ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
        ser.baudrate = 38400
        ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
        ser.connected = False
        while not ser.connected:
            try:
                ser.open()
            except serial.SerialException as e:
                print(f"Error: {e}")
            else:
                ser.connected = True
            time.sleep(.5)


        # Loop forever
        while True:

            received = json.loads(ser.readline().decode())
            frame = fetch_frame(vs)
            with lock:
                # Write received data dict to global variable with lock
                data_dict = copy.deepcopy(received)

                # Write frame to dict, since it's only available to the pi
                data_dict["frame"] = copy.deepcopy(frame)

    def queue_write(queue=None):
        global data_dict, lock

        while True:
            # Write data dict to queues
            # Put data dictionary into queue if queue isn't full
            if queue is not None:
                if not queue.full():
                    with lock:
                        myput(queue, data_dict)
                            
            # Break if no queues
            else:
                break

    def server_write(local_server=False):
        global data_dict, lock

        # Define server ip, port, and client
        server_port = 9999
        if local_server:
            host_ip = "127.0.0.1"
        else:
            host_ip = "192.168.1.52"
        udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while True:

            with lock:
                send_dict = copy.deepcopy(data_dict)

            if send_dict["frame"] is not None:
                send_dict["frame"] = send_dict["frame"].tolist()
            send_msg = json.dumps(send_dict).encode()

            #mysendto(udp_client, send_msg, host_ip, server_port)
            udp_client.sendto(send_msg, (host_ip, server_port))

    def myput(queue, obj):
        if queue.full():
            while not queue.empty():
                try:
                    queue.get(block=False)
                except Exception as e:
                    print(f"Error: {e}")
                finally:
                    queue.put(obj)

    def mysendto(udp_client, message, host_ip, server_port):
        # UDP messages cap out at a certain length. This allows us to fragment and send
        #   any size of UDP message to avoid the byte cap (hitting it right now at 13855 bytes)

        # Define max fragment size
        frag_size = 1000

        # Measure number of packets that need to be sent if message fragments are size 1000
        n_packets = math.ceil(len(message) / frag_size)

        # Tell the server the number of fragments to be sent
        udp_client.sendto({'n_packets': n_packets}, (host_ip, server_port))

        # Fragment the message and send each
        while len(message) != 0:

            if len(message) > frag_size:
                udp_client.sendto({"message": message[0:frag_size]}, (host_ip, server_port))
                message = message[frag_size:len(message)]
            else:
                udp_client.sendto({"message": message}, (host_ip, server_port))
            

    threads = []
    threads.append(threading.Thread(target=sensor_read, args=(picam,)))
    threads.append(threading.Thread(target=queue_write, args=(write_queue,)))
    threads.append(threading.Thread(target=server_write, args=(local_server,)))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()


if __name__ == "__main__":
    main()





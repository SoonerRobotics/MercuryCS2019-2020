#!/usr/bin/env python3

# Pipeline:
#   Input from arduino sensors (blocking) and pi camera
#   Output to UI (on udp client)
#   Output to nav (on queue)
#   Output to obstacle avoidance (on queue)

import sys, serial, json, socket, datetime, time, threading
import imutils, cv2

def fetch_frame(vs):

    # Grab a frame from the camera stream
    frame = vs.read()
    frame = imutils.resize(frame, width=400)

    # grab the current timestamp and draw it on the frame
    timestamp = datetime.datetime.now()
    cv2.putText(frame, timestamp.strftime(
        "%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

    return frame

def sensor_read():
    global data_dict, lock

    # Start up camera
    vs = VideoStream(usePiCamera=1).start()
    #vs = VideoStream(src=0).start()
    time.sleep(2.0)

    # Define serial connection
    ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
    ser.baudrate = 38400
    ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
    ser.open()

    # Loop forever
    while True:

        with lock:
            # Write received data dict to global variable with lock
            data_dict = json.loads(ser.readline().decode())

            # Write frame to dict, since it's only available to the pi
            data_dict["frame"] = fetch_frame(vs)

def queue_write(queues=[]):
    global data_dict, lock

    while True: 
        # Write data dict to queues
        # Put data dictionary into queue if queue isn't full
        if queues:
            for queue in queues:
                if not queue.full():
                    with lock:
                        queue.put(data_dict)
                        
        # Break if no queues
        else:
            break

def server_write():
    global data_dict, lock

    # Define server ip, port, and client
    host_ip, server_port = "192.168.1.52", 9999
    udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:

        with lock:
            # Write data dict to server
            udp_client.sendto(json.dumps(data_dict).encode(), (host_ip, server_port))

def main(queues=[]):

    # Define data dict to be send
    data_dict = {
        "frame": None,
        "us1": None,
        "us2": None,
        "us3": None,
        "a": None,
        "m": None
    }

    # Define thread lock
    lock = threading.Lock()

    threads = []
    threads.append(threading.Thread(target=sensor_read))
    threads.append(threading.Thread(target=queue_write, args=(queues,)))
    threads.append(threading.Thread(target=server_write))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()


if __name__ == "__main__":
    main()





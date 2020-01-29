#!/usr/bin/env python3

import sys, serial, json, socket, threading, copy, time

def main(receive_sensors=None, receive_nav=None):
    
    global data_dict, lock, control_dict, autonomous_signal, idle_dict, safe_distance

    # Define data dict to be send
    data_dict = {
        "frame": None,
        "us1": None,
        "us2": None,
        "us3": None,
        "a": None,
        "m": None
    }

    # Define idle
    idle_dict = {
        "left_motor": 0,
        "right_motor": 0,
        "arm_motor": 0
    }

    # Define control dict to send
    control_dict = copy.deepcopy(idle_dict)

    # Thread lock
    lock = threading.Lock()

    # Whether or not the bot listens to the nav or the controller
    autonomous_signal = False

    def sensors_read(receive_queue=None):
        global data_dict, lock

        while True:

            if receive_queue is not None:
                if not receive_queue.empty():
                    received = receive_queue.get()
                    with lock:
                        data_dict = copy.deecopy(received)

    def nav_read(receive_queue=None):
        global control_dict, lock

        while True:

            if receive_queue is not None:
                if not receive_queue.empty() and autonomous_signal:
                    received = receive_queue.get()
                    with lock:

                        # Autonomous signal controlled only by controller
                        for key in received.keys():
                            try:
                                control_dict[key] = copy.deepcopy(received[key])
                            except KeyError as e:
                                pass

    def controller_read():
        global control_dict, lock

        # Define server ip, port, and client
        host_ip, server_port = "192.168.1.52", 9999
        udp_client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while True:
            received = udp_client.recv(1024)
            with lock:
                control_dict = json.loads(received.decode())

    def motor_write():
        global control_dict, lock, idle_dict

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

        while True:

            with lock:
                send_dict = copy.deepcopy(control_dict)

            # If safe path
            if collision_filter(send_dict):
                ser.write(json.dumps(send_dict).encode())
            # Else write idle
            else:
                ser.write(json.dumps(idle_dict).encode())

    def collision_filter(control_dict):
        # Dummy function for now

        global data_dict, safe_distance, lock

        return True

    def myput(queue, obj):

        try:
            queue.put_nowait(obj)
        except Exception as e:
            pass
                

    threads = []
    threads.append(threading.Thread(target=sensors_read, args=(receive_sensors,)))
    threads.append(threading.Thread(target=nav_read, args=(receive_nav,)))
    threads.append(threading.Thread(target=controller_read))
    threads.append(threading.Thread(target=motor_write))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()



if __name__ == "__main__":
    main()
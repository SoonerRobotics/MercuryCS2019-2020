#!/usr/bin/env python3

import sys, serial, json, socket, threading, copy, time, logging

import help_lib as hl

def main(receive_sensors=None, receive_nav=None, local_server=False):
    """Switch between controls from navigation and controller, then feed to Arduino"""
    
    global data_dict, lock, control_dict, autonomous_signal, safe_distance, logger, c_status

    # Create logger
    logger = hl.create_logger(__name__)

    # Initial status
    c_status = "Disconnected"

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
    control_dict = {
        "left_stick": 0,
        "right_stick": 0,
        "led": False
    }

    # Thread lock
    lock = threading.Lock()

    # Whether or not the bot listens to the nav or the controller
    autonomous_signal = False

    def sensors_read(receive_queue=None):
        """Read sensors from sensor relay"""
        global data_dict, lock

        while True:

            if receive_queue is not None:
                if not receive_queue.empty():
                    received = receive_queue.get()
                    with lock:
                        data_dict = copy.deepcopy(received)

    def nav_read(receive_queue=None):
        """Read controls from navigation"""
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

    def controller_read(local_server=False):
        """Read controls from controller"""
        global control_dict, lock, autonomous_signal, logger

        # Define server ip, port, and client
        server_port = 9001
        if local_server:
            host_ip = "localhost"
        else:
            host_ip = "192.168.1.52"

        # Attempt to connect to server until successful
        logger.info(f"Client Target Address: {host_ip}:{server_port}")
        while True:
            try:
                tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                tcp_client.connect((host_ip, server_port))
                c_status = "Connected"
                logger.info(c_status)
            except Exception as e:
                logger.warn(f"Error: {e}")

            # While connected, read input from controller and write to control dictionary
            while c_status == "Connected":

                try:
                    receive_dict = tcp_client.makefile().readline()
                except Exception as e:
                    c_status = "Disconnected"
                    logger.warn(e)
                    continue

                with lock:
                    autonomous_signal = receive_dict.pop("autonomous_signal")

                if not autonomous_signal:
                    with lock:
                        control_dict = copy.deepcopy(receive_dict)

    def motor_write():
        """Write controls to the motor"""
        global control_dict, lock, logger

        # Define idle
        idle_dict = {
            "left_stick": 0,
            "right_stick": 0,
            "led": False
        }

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

        # While connected, write filtered control dictionary to Arduino
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
        """Decide if the control dictionary causes a collision"""

        global data_dict, safe_distance, lock

        return True

    # Spin up threads
    threads = []
    threads.append(threading.Thread(target=sensors_read, args=(receive_sensors,)))
    threads.append(threading.Thread(target=nav_read, args=(receive_nav,)))
    threads.append(threading.Thread(target=controller_read, args=(local_server,)))
    threads.append(threading.Thread(target=motor_write))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()

if __name__ == "__main__":
    main()
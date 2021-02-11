#!/usr/bin/env python3

import sys, json, socket, threading, copy, time

import serial

import help_lib as hl

def main(receive_sensors=None, write_queue=None):
    """Create autonomous navigation controls from sensors"""
    
    global data_dict, lock, state, control_dict, logger

    # Create logger
    logger = hl.create_logger(__name__)

    # Define data dict to be sent
    data_dict = {
        "frame": None,
        "us1": None,
        "us2": None,
        "us3": None,
        "a": None,
        "m": None
    }

    # Define control dict to send
    control_dict = {
        "left_stick": 5,
        "right_stick": 5,
        "led": False
    }

    # Thread lock
    lock = threading.Lock()

    # State machine state. Starts at 0. There will likely be one state for every turn. Maybe 2 extra states for the pre-start
    #   and end state.
    state = -1

    # Read from sensor relay
    def sensors_read(receive_queue=None):
        """Read sensors from sensor relay"""
        global data_dict, lock, logger

        while True:

            if receive_queue is not None:
                if not receive_queue.empty():
                    received = receive_queue.get()

                    with lock:
                        data_dict = copy.deepcopy(received)
            else:
                break

    def queue_write(queue=None):
        """Write to multiprocess output queue"""
        global data_dict, lock, control_dict

        while True:

            if queue is not None:
                with lock:
                    hl.myput(queue, control_dict)
            else:
                break

    def state_machine():
        """State machine for known autonomous navigation"""
        global data_dict, lock, state, control_dict

        while True:

            if state == -1:
                continue

    # Spin up threads
    threads = []
    threads.append(threading.Thread(target=sensors_read, args=(receive_sensors,)))
    threads.append(threading.Thread(target=queue_write, args=(write_queue,)))
    threads.append(threading.Thread(target=state_machine))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()



if __name__ == "__main__":
    main()
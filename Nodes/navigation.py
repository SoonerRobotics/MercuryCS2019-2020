#!/usr/bin/env python3

import sys, serial, json, socket, threading, copy, time

def main(receive_sensors=None, write_queue=None):
    
    global data_dict, lock, state, control_dict

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
        "left_motor": 5,
        "right_motor": 5,
        "arm_motor": 5
    }

    # Thread lock
    lock = threading.Lock()

    # State machine state. Starts at 0. There will likely be one state for every turn. Maybe 2 extra states for the pre-start
    #   and end state.
    state = -1

    # Read from sensor relay
    def sensors_read(receive_queue=None):
        global data_dict, lock

        while True:

            if receive_queue is not None:
                if not receive_queue.empty():
                    received = receive_queue.get()
                    print(f"Nav: val received {received}")
                    with lock:
                        data_dict = copy.deepcopy(received)
            else:
                break

    def queue_write(queue=None):
        global data_dict, lock, control_dict

        while True:

            if queue is not None:
                with lock:
                    myput(queue, control_dict)

    def state_machine():
        global data_dict, lock, state, control_dict

        while True:

            if state == -1:
                continue

    def myput(queue, obj):

        try:
            queue.put_nowait(obj)
        except Exception as e:
            pass

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
#!/usr/bin/env python3

import sys, serial, json, socket, threading, copy

def main(receive_sensors=None, write_queue=None):
    
    global data_dict, lock, state, control_dict

    # Define data dict to be send
    data_dict = {
        "frame": None,
        "us1": None,
        "us2": None,
        "us3": None,
        "a": None,
        "m": None
    }

    # Define control dict to send
    control_dict = {"null": "null"}

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
        if queue.full():
            while not queue.empty():
                try:
                    queue.get(block=False)
                except Exception as e:
                    print(f"Error: {e}")
                finally:
                    queue.put(obj)

    threads = []
    threads.append(threading.Thread(target=sensors_read, args=(receive_sensors,)))
    threads.append(threading.Thread(target=queue_write, args=(write_queues,)))
    threads.append(threading.Thread(target=state_machine))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()



if __name__ == "__main__":
    main()
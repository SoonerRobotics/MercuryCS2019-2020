#!/usr/bin/env python3

import multiprocessing

from Nodes import sensor_relay
from Nodes import navigation
from Nodes import collision_avoidance
from Nodes import controller_handler
from Nodes import ui

def main():

    # Create queues
    sensor_relay_producer = multiprocessing.Queue(maxsize=1)
    navigation_producer = multiprocessing.Queue(maxsize=1)

    processes = []
    # Start sensor_relay.py
    processes.append(multiprocessing.Process(target=sensor_relay.main, 
        kwargs={"write_queue": sensor_relay_producer,
                "local": True}))

    # Start naviation.py
    processes.append(multiprocessing.Process(target=navigation.main,
        kwargs={"receive_sensors": sensor_relay_producer,
                "write_queue": navigation_producer}))

    # Start collision_avoidance.py
    processes.append(multiprocessing.Process(target=collision_avoidance.main,
        kwargs={"receive_sensors": sensor_relay_producer, 
                "receive_nav": navigation_producer,
                "local": True}))
    
    # Start controller_handler.py
    processes.append(multiprocessing.Process(target=controller_handler.main,
        kwargs={"local": True}))

    # Start ui.py
    processes.append(multiprocessing.Process(target=ui.main,
        kwargs={"local": True}))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == "__main__":
    main()
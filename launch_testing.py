#!/usr/bin/env python3

import multiprocessing

from Nodes import controller_handler
from Nodes import ui

def main():

    processes = []
    # Start sensor_relay.py
    processes.append(multiprocessing.Process(target=sensor_relay.main, args=(sensor_relay_producer,)))

    # Start naviation.py
    processes.append(multiprocessing.Process(target=navigation.main, args=(sensor_relay_producer, navigation_producer)))

    # Start collision_avoidance.py
    processes.append(multiprocessing.Process(target=collision_avoidance.main, args=(sensor_relay_producer, navigation_producer)))
    
    # Start controller_handler.py
    processes.append(multiprocessing.Process(target=controller_handler.main))

    # Start ui.py
    processes.append(multiprocessing.Process(target=ui.main))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == "__main__":
    main()
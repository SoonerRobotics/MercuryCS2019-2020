#!/usr/bin/env python3

import multiprocessing

from Nodes import sensor_relay
from Nodes import navigation
from Nodes import collision_avoidance

def main():
    
    # Create queues
    sensor_relay_producer = multiprocessing.Queue(maxsize=1)
    navigation_producer = multiprocessing.Queue(maxsize=1)

    processes = []
    # Start sensor_relay.py
    processes.append(multiprocessing.Process(target=sensor_relay.main, args=(sensor_relay_producer,)))

    # Start naviation.py
    processes.append(multiprocessing.Process(target=navigation.main, args=(sensor_relay_producer, navigation_producer)))

    # Start collision_avoidance.py
    processes.append(multiprocessing.Process(target=collision_avoidance.main, args=(sensor_relay_producer, navigation_producer)))

    for process in processes:
        process.start()

    for process in processes:
        process.join()

if __name__ == "__main__":
    main()
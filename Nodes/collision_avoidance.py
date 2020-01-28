#!/usr/bin/env python3

import threading

def main(receive_sensors=None, receive_nav=None, receive_controller=None):
	
	global data_dict, lock, control_dict, autonomous_signal

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

    # Whether or not the bot listens to the nav or the controller
    autonomous_signal = False

	def sensors_read(receive_queue=None):
		global data_dict, lock

		while True:

			if receive_queue is not None:
				if not receive_queue.empty():
					with lock:
						data_dict = receive_queue.get()

	def nav_read(receive_queue=None):
		global control_dict, lock

		while True:

			if receive_queue is not None:
				if not receive_queue.empty() and autonomous_signal:
					with lock:
						control_dict = receive_queue.get()

	def controller_read(receive_queue=None):
		global control_dict, lock

		while True:

			if receive_queue is not None:
				if not receive_queue.empty() and not autonomous_signal:
					with lock:
						control_dict = receive_queue.get()

	def motor_write():
		global control_dict, lock

		ser = serial.Serial(timeout = 1) # Set serial timeout to 1 second
		ser.baudrate = 38400
		ser.port = "/dev/ttyUSB0" # TODO: use try/catch to find the port Arduino is connected to automatically
		ser.open()

		while True:

			with lock:
				ser.write(json.dumps(control_dict).encode())
				

	threads = []
    threads.append(threading.Thread(target=sensors_read, args=(receive_sensors)))
    threads.append(threading.Thread(target=nav_read, args=(receive_nav)))
    threads.append(threading.Thread(target=controller_read, args=(receive_controller)))
    threads.append(threading.Thread(target=motor_write))

    for i in range(len(threads)):
        threads[i].daemon = True
        threads[i].start()

    for i in range(len(threads)):
        threads[i].join()



if __name__ == "__main__":
	main()
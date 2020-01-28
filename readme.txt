Robot software layout
----------------------
• Arduino 1
	• sensor_driver.ino
		• Purpose: Run sensors and output their values to the pi
		• Output: sensor_relay.py through serial
• Arduino 2
	• motor_driver.ino
		• Purpose: Run motors and perform collision avoidance
		• Input: navigation.py through serial
• Laptop
	• launch_laptop.py
		• Purpose: launch and link all laptop programs
	• ui.py
		• Purpose: Display data incoming through pi
		• Input: sensor_relay.py through upd_client
	• controller_handler.py
		• Purpose: Read controller values and output the controls to the pi
		• Output: controller_relay.py through upd_client
• Pi
	• launch_pi.py
		• Purpose: launch and link all pi programs
	• sensor_relay.py
		• Purpose: Relay the sensor values from the arduino to all the relevant programs. If wifi drops,
			continues to send on queues.
		• Input: sensor_driver.ino through serial
		• Output: ui.py through upd_client
		• Output: naviation.py through queue
		• Output: collision_avoidance.py
	• navigation.py
		• Purpose: During known autonomous section, performs state machine dead reckoning. During unknown
			autonomous section, performs unknown.
		• Input: sensor_relay.py through queue
		• Output: collision_avoidance.py through queue
	• collision_avoidance.py
		• Purpose: Route incoming control values from navigation.py and controller_handler.py. By default, ignores navigation.py
			control values. If autonomous start signal is sent, follows navigation.py controls. If autonomous stop signal is sent,
			follows controller_handler.py controls. If connection is dropped and autonomous isn't set, turn on idle. If connection
			is dropped and autonomous signal is set, follow navigation.py controls.
		• Input: navigation.py through queue
		• Input: controller_handler.py through queue
		• Output: motor_driver.ino through serial
#!/usr/bin/env python

import rospy, copy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from game_manager.msg import ButtonState, TowerState, TiltSensor, Towers
from game_manager.srv import SetLEDs

class Tower():
	NUM_CHARGE_LEDS = 4
	ALLOWED_STATUSES = [TowerState.TYPE_TOWER_CAPTURED,
						TowerState.TYPE_TOWER_HAS_FALLEN,
						TowerState.TYPE_TOWER_OFFLINE,
						TowerState.TYPE_TOWER_ONLINE]
	
	LED_COLOR = {'red':(1,0,0), 'green': (0,1,0), 'blue': (0,0,1), 'blank': (0,0,0)}

	def __init__(self, id, service_name):
		# the tower id
		self._id = id
		# the led ros service name from which to communicate led statuses.
		self._service_name = service_name
		# the current tower state (see Tower.ALLOWED_STATUSES static variable)
		self._status = TowerState.TYPE_TOWER_OFFLINE
		# the current charge led statuses
		self._leds = [0 for i in range(Tower.NUM_CHARGE_LEDS)]
		# the current number of button presses 
		self._num_presses = 0
		# the current color for the tower status led
		self._status_led_color = Tower.LED_COLOR['blank']
	
	@property
	def id(self):
		return self._id
	
	@property
	def service_name(self):
		return self._service_name

	@property
	def status(self):
		return self._status
	
	@status.setter
	def status(self, value):
		if value in Tower.ALLOWED_STATUSES:
			self._status = value
	
	@property
	def leds(self):
		return self._leds

	@property
	def status_led_color(self):
		return self._status_led_color

	@status_led_color.setter
	def status_led_color(self, value):
		if value in Tower.LED_COLOR.values():
			self.status_led_color = value
	
	@property
	def num_presses(self):
		return self._num_presses

	def num_presses_add_one(self):
		""" Increase number of _num_presses by one.
		To be called each time the button is pressed."""
		self._num_presses +=1
	
	def update_led(self, id_led, state):
		""" 
		Update led state
		Params
			id_led (int, range: 1-NUM_CHARGE_LEDS):	the charge led number
			state (bool):	the new state for the led
			"""
		self._leds[id_led] = state

	def num_ON_leds(self):
		""" Returns the number of charge leds turned ON by the player"""
		return sum(self._leds)

	def get_data_as_ros_msg(self):
		"""Return the data as ROS TowerState msg"""
		data = TowerState()
		data.id = self.id
		data.status = self.status
		data.leds = self.leds
		data.bt_presses = self.num_presses
		return data

	def reset_data(self):
		"""Reset data to init values"""
		self._status = TowerState.TYPE_TOWER_OFFLINE
		self._leds = [0 for i in range(Tower.NUM_CHARGE_LEDS)]
		self._num_presses = 0
		self._status_led_color = Tower.LED_COLOR['blank']


class GameManagerNode:
	
	def __init__(self):
		# init node
		rospy.init_node('game_manager')
		
		# setup subscriber 
		rospy.Subscriber('/joy', Joy, self.joy_callback)

		rospy.Subscriber(rospy.get_param("/button_topic_name"), ButtonState,  self.tower_bt_callback)
		rospy.Subscriber(rospy.get_param("/tilt_sensor_topic_name"), TiltSensor,  self.tower_tilt_sensor_callback)
		
		# setting towers
		self.service_names = rospy.get_param("/tower_service_names")
		self.towers = dict((int(i),0) for i in self.service_names.keys())
		for id, service_name in self.service_names.iteritems():
			self.towers[int(id)] = Tower(int(id),service_name)

		self.BT_PRESS_THD = rospy.get_param("/charging_time")
		self.CHAR_LED_PER_TW = rospy.get_param("/num_charge_leds_per_tower")
		self.NUM_TOWERS = rospy.get_param("/num_towers")
		self.FEEDBACK_DURATION =  rospy.get_param("/led_feedback_duration")

		# This publisher is used to signal the game_start
		self.pub_game_on = rospy.Publisher('game_manager/isGameON', Bool, queue_size=1)
		# This publisher is used to signal the game_start
		self.pub_tws_state = rospy.Publisher('game_manager/towers/State', Towers, queue_size=10)
		
		# ros rate
		self.rate = rospy.Rate(10) # 10hz

		# 
		self.feedback_timer = None
		self.feedback_status = False

		self.trying_reset = False

		self.bt_msg_on_process   = None
		
		self.srv_handlers = dict((i,None) for i in self.towers.keys())
		self.srv_error_list = []
		self.sub_to_services()

		self.reset_game()

	def sub_to_services(self, com_timeout=3):
		""" Subscribe to all tower LED services
			Params
				com_timeout wait timeout
			Return
				None
		"""
		retry = False
		while not retry and not rospy.is_shutdown():
			for i in self.towers.keys():
				
				if self.srv_handlers[i] is not None:
					continue
				else:
					if i not in self.srv_error_list:
						rospy.loginfo("Trying to contact {} service...".format(self.towers[i].service_name))
					else:
						rospy.loginfo("Retrying contact with {} service...".format(self.towers[i].service_name))
					try:
						rospy.wait_for_service(self.towers[i].service_name, timeout=com_timeout)
						rospy.loginfo("SUCCESS... establishing connection to {}...".format(self.towers[i].service_name))
						self.srv_handlers[i] = rospy.ServiceProxy(self.towers[i].service_name, SetLEDs)
						rospy.loginfo("SUCCESS... connection with {} established!".format(self.towers[i].service_name))
						self.towers[i].status = TowerState.TYPE_TOWER_ONLINE
						if i in self.srv_error_list:
							self.srv_error_list.remove(i)
							
					except rospy.exceptions.ROSException as exc:
						rospy.logerr("TIMEOUT: Service {} seems to be offline! Retrying soon...".format(self.towers[i].service_name))
						self.srv_error_list.append(i)
			retry = True if not len(self.srv_error_list) else False

	def tower_bt_callback(self, msg):
		"""Tower button callback"""

		rospy.loginfo("Button state from tower #{} is {}!".format(msg.id,msg.value))

		if self.towers[msg.id].status == TowerState.TYPE_TOWER_CAPTURED or \
			self.towers[msg.id].status == TowerState.TYPE_TOWER_HAS_FALLEN:
			return

		if msg.value:
			self.bt_msg_on_process = msg
			self.towers[msg.id].num_presses_add_one()    # add press counter for the corresponding tower
		elif msg.value==False and self.bt_msg_on_process is not None:
			time_diff = msg.header.stamp.to_sec() - self.bt_msg_on_process.header.stamp.to_sec()
			if time_diff < 0: rospy.logerr("Time diff bettween pressing and unpressing button cannot be negative!")
			if (time_diff / self.BT_PRESS_THD) >= 1:
				self.update_tw_led(self.bt_msg_on_process.id)
			else:
				rospy.loginfo("Time elapsed from last led: {:.2f}".format(time_diff))
			self.bt_msg_on_process = None

	def tower_tilt_sensor_callback(self, msg):
		"""Tilt sensor callback"""
		self.towers[msg.id].status(TowerState.TYPE_TOWER_HAS_FALLEN
								   if msg.value else TowerState.TYPE_TOWER_ONLINE)
	
	def checkCapture(self, tw_id):
		"""
		Checks wether player has captured the tower
		Params
			tw_id (int) :	tower id
		Return
			None
		"""
		return  self.towers[tw_id].num_ON_leds() == self.CHAR_LED_PER_TW


	def start_feedback_timer(self):
		"""Starts a timer for blinking the status led on tower, giving the player a feedback
		when pressing the button"""
		self.feedback_timer = rospy.Timer(rospy.Duration(self.FEEDBACK_DURATION),
										  self.feedback_callback)
	
	def stop_feedback_timer(self, parameter_list):
		"""Stops a timer for blinking the status led on tower"""
		self.feedback_timer.shutdown()
		self.feedback_timer = None

	def feedback_callback(self, event):
		self.send_update_to_service(self.bt_msg_on_process.id)

	def update_tw_led(self, tower_number):
		old_num_ON = self.towers[tower_number].num_ON_leds()
		self.towers[tower_number].update_led(old_num_ON, 1)
		self.send_update_to_service(tower_number)
		rospy.logwarn("#leds in tower{}: {}".format(tower_number,self.towers[tower_number].leds))
		rospy.logwarn("Bt_presses in tower{}: {}".format(tower_number,self.towers[tower_number].num_presses))
	
	def talk_to_service(self, tw_id, num_LED_ON, status_LED_color):
		"""
		Calls the tower led service
			Params
				tw_id :	the tower id
            	req : a request msg
            	resp : the response msg container
        	Returns
            	None
		"""
		try:
			handle = self.srv_handlers[tw_id]
			resp = handle(num_LED_ON, status_LED_color)
			return resp
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e


	def joy_callback(self, msg):
		"""
		Reads the joy message and checks whether start button was pressed.
		"""
		if msg.buttons[5] and msg.buttons[9] == 1 and not trying_reset:
			self.trying_reset = not trying_reset
			rospy.loginfo("Trying to reset towers AND start game..")
			sucess = reset_game()
			msg = Bool()
			msg.data = self.started
			self.pub_game_on.publish(msg)

	def reset_game(self):
		"""Reset game by reseting tower data"""
		for tw in self.towers.keys():
			self.towers[tw].reset_data()
			if not self.send_update_to_service(tw):
				rospy.signal_shutdown("Could not reset game due to bad response from tower service!")
			else:
				rospy.loginfo("Game was reset! ")
		
		self.trying_reset = False
		return True

	def count_time(self):
		"""Counts second past button presses"""
		if self.bt_msg_on_process is not None:

			if self.checkCapture(self.bt_msg_on_process.id):
				self.towers[self.bt_msg_on_process.id].status = TowerState.TYPE_TOWER_CAPTURED
				self.bt_msg_on_process = None
			else:
				now = rospy.Time.now()
				time_diff = now.to_sec() - self.bt_msg_on_process.header.stamp.to_sec()
				if time_diff < 0: rospy.logerr("count_time(): Time diff bettween pressing and unpressing button cannot be negative!")
				if (time_diff / self.BT_PRESS_THD) >= 1 and self.towers[self.bt_msg_on_process.id].status == TowerState.TYPE_TOWER_ONLINE:
					self.update_tw_led(self.bt_msg_on_process.id)
					self.bt_msg_on_process.header.stamp = now
				

	def send_update_to_service(self, tw_id):
		""" Sends the new LED state to towers """
		try:
			resp = self.talk_to_service(tw_id, self.towers[tw_id].leds[:3], Tower.LED_COLOR['blank'])
			rospy.logwarn("send_update_to_service(): Response from {}: {}".format(self.towers[tw_id].service_name,
														'SUCCESS' if resp else 'FAILURE'))
			return resp
		except Exception as exc:
			rospy.logerr("Could not talk to {} service!".format(self.towers[tw_id].service_name))

	def publish_state_of_towers(self):
		"""Publish state of towers"""
		msg = Towers()
		msg.header.stamp = rospy.Time.now()
		for i in self.towers.keys():
			setattr(msg, "tw"+str(i), self.towers[i].get_data_as_ros_msg())
		self.pub_tws_state.publish(msg)

	def run(self):
		"""main loop"""
		while not rospy.is_shutdown():
			self.count_time()
			self.publish_state_of_towers()
			self.rate.sleep()
			

if __name__ == '__main__':
	node = GameManagerNode()
	node.run()


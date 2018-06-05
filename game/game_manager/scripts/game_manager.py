#!/usr/bin/env python

import rospy
import os.path
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
from record_ros.srv import String_cmd
from arduino_publisher.msg import TowerState


RESET_THD = 5  # in secs
N_TOWER = 4
tower_status = [0 for x in range(N_TOWER)]

to_start = False
time_since_to_start = None
started = False

def towerCallback(data):
	global tower_status, started

	# if (started):
	# 	if (data.status == -1):
	# 		status = False
	# 	else:

	if (sum(data.leds) == 0):
		tower_status[data.id-1] = 1



def joyCallback(data):
	global to_start, time_since_to_start
	if data.buttons[5] and data.buttons[9] == 1 and not to_start:
		to_start = not to_start
		if to_start:
			rospy.loginfo("Trying to reset towers AND start game..")
			time_since_to_start = rospy.get_rostime()
	elif data.buttons[5] and data.buttons[9] == 1 and to_start:
		to_start = False
		rospy.loginfo("Stopping..")

		
		


def gameManagerNode():
	global tower_status,to_start, time_since_to_start

	#Init node
	rospy.init_node('game_manager')
	
	#Setup subscriber 
	rospy.Subscriber('/joy', Joy, joyCallback)
	#Setup subscriber 
	rospy.Subscriber('/arduino/tower_state', TowerState, towerCallback)

	#Setup publisher

	# This publisher is used to signal the game_start
	pub_game_on = rospy.Publisher('game_manager/isGameON', Bool, queue_size=1)
	
	rate = rospy.Rate(10) # 10hz

	time_since_to_start = rospy.get_rostime()
	started = False

	while not rospy.is_shutdown():
		rospy.loginfo_throttle(3, "Tower ON Status: {}".format(tower_status))
		if (to_start and not started):
			if (sum(tower_status) == 4):
				rospy.loginfo("Game is ready to go! TOWER RED LED SHOULD BE STILL!")
				started = True
			else:	# continue trying to reset towers
				if (rospy.get_rostime().secs - time_since_to_start.secs > RESET_THD):
					rospy.logwarn("Taking too long to successfully confirm all towers are reset! Are you sure they are ALL turned on? RETRYING...")

		elif not to_start and started:
			rospy.loginfo("Stoping game! TOWER RED LED SHOULD BE BLINKING!")
			started = False
			tower_status = [0 for x in range(N_TOWER)]
		
		msg = Bool()
		msg.data = started
		pub_game_on.publish(msg)

		rate.sleep()
			

if __name__ == '__main__':
	gameManagerNode()


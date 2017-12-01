#!/usr/bin/env python

import rospy
import os.path
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from record_ros.srv import String_cmd

isRecording = False
isPressed = False
#Setup publisher
#pub = rospy.Publisher('robogame/gameState', interState, queue_size=10)
	

def joyCallback(data):
	global isRecording
	global isPressed
	if data.buttons[5] and data.buttons[8] == 1 and not isPressed:
		rospy.loginfo('Start button pressed')
		rospy.wait_for_service('/record/cmd')
		try:
			rospy.loginfo('Sending request')
			service = rospy.ServiceProxy('/record/cmd', String_cmd)
			if not isRecording:
				rospy.loginfo('Recording...')
				service('record')
				isRecording = True
				## PUBLISH LED MESSAGE
				#pub.publish(msg)
			else:
				rospy.loginfo('Stop recording');
				service('stop')
			
		except rospy.ServiceException, e:
			isRecording = False
			## PUBLISH LED MESSAGE HERE
		isPressed = True
	elif data.buttons[5] and data.buttons[8] == 0:
		isPressed = False


def statusNode():
	#Init node
	rospy.init_node('status', anonymous=True)
	
	#Setup subscriber 
	rospy.Subscriber('/joy', Joy, joyCallback)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
			

if __name__ == '__main__':
	statusNode()


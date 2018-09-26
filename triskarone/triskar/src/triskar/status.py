#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import os.path
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from record_ros.srv import String_cmd

ON = 11
SERIAL_UP = 13
RECORDING = 15

isRecording = False
isPressed = False

def joyCallback(data):
	global isRecording
	global isPressed
	global startButton
	if data.buttons[startButton] == 1 and not isPressed:
		print 'start button pressed'
		rospy.wait_for_service('/record/cmd')
		try:
			print 'sending request'
			service = rospy.ServiceProxy('/record/cmd', String_cmd)
			if not isRecording:
				print'record'
				service('record')
				isRecording = True
				GPIO.output(RECORDING, True)
			else:
				print 'stop'
				service('stop')
			
		except rospy.ServiceException, e:
			isRecording = False
			GPIO.output(RECORDING, False)
		isPressed = True
	elif data.buttons[startButton] == 0:
		isPressed = False


def statusNode():
	global startButton
	#Init node
	rospy.init_node('status', anonymous=True)
	
	#Setup subscriber 
	rospy.Subscriber('/joy', Joy, joyCallback)

	#Setup parameters
	startButton = rospy.get_param('startButton', 9)
	
	#Init GPIO
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(ON, GPIO.OUT)
	GPIO.setup(SERIAL_UP, GPIO.OUT)
	GPIO.setup(RECORDING, GPIO.OUT)
	
	#Enable ON led, disable others
	GPIO.output(ON, True)
	GPIO.output(SERIAL_UP, False)
	GPIO.output(RECORDING, False)
	
	rate = rospy.Rate(1) # 1hz
	
	serialDevice = '/dev/ttyACM0'
	
	while not rospy.is_shutdown():
		if os.path.exists(serialDevice):
			GPIO.output(SERIAL_UP, True)
		else:
			GPIO.output(SERIAL_UP, False)
		rate.sleep()
			

if __name__ == '__main__':
	try:
		statusNode()
		        
	except rospy.ROSInterruptException:
		pass
	GPIO.output(ON, False)
	GPIO.output(SERIAL_UP, False)
        GPIO.output(RECORDING, False)
	GPIO.cleanup()

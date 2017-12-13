#!/usr/bin/env python
import rospy
import os.path
from std_msgs.msg import String, Bool
from record_ros.srv import String_cmd

isRecording = False
isStarted = False

def recordCallback(msg):
	global isRecording
	global isStarted
	if msg.data and not isStarted:
		rospy.loginfo('Starting signal received')
		rospy.wait_for_service('/video_record/video_rec_cmd')
		try:
			rospy.loginfo('Sending request')
			service = rospy.ServiceProxy('/video_record/video_rec_cmd', String_cmd)
			if not isRecording:
				rospy.loginfo('Recording...')
				service('record_video')
				isRecording = True
		except rospy.ServiceException, e:
			isRecording = False
		isStarted = True
	elif not msg.data and isStarted:
		isStarted = False
		try:
			rospy.loginfo('Sending request for stop recording')
			rospy.wait_for_service('/video_record/video_rec_cmd')
			service = rospy.ServiceProxy('/video_record/video_rec_cmd', String_cmd)
			service('stop_video')
		except rospy.ServiceException, e:
			#TODO check why this keeps being executed upon stop_video reques.
			# ERROR: From video_recorder: transport error completing service call: 
			#  		 unable to receive data from sender, check sender's logs for details
			rospy.logerr("From video_recorder: {}".format(e))
		isRecording = False

def statusNode():
	#Init node
	rospy.init_node('external_video_bag_recorder')
	
	#Setup subscriber 
	rospy.Subscriber('/logging/record_ext_video', Bool, recordCallback)
	
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
			

if __name__ == '__main__':
	statusNode()


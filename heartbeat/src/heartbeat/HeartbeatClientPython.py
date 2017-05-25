#!/usr/bin/env python
import rospy
from heartbeat.srv import RegisterNode
from heartbeat.srv import UnregisterNode
from heartbeat.srv import SetState
from heartbeat.msg import State, Heartbeat
from random import randint
from copy import deepcopy

class HeartbeatClientPy:
	def __init__(self):
		self.state = 0
		self.node_name = ''
		self.pub = rospy.Publisher('heartbeat', Heartbeat, queue_size=10)

		rospy.wait_for_service('/heartbeat/register_node')
		try:
			self.register_node = rospy.ServiceProxy('/heartbeat/register_node', RegisterNode)
			print "Service register_node call"
		except rospy.ServiceException, e:
			print "Service call failed: %s,"%e	
		
		rospy.wait_for_service('/heartbeat/unregister_node')
		try:
			self.unregister_node = rospy.ServiceProxy('/heartbeat/unregister_node', UnregisterNode)
			print "Service unregister_node call"
		except rospy.ServiceException, e:
			print "Service call failed: %s,"%e
		rospy.wait_for_service('/heartbeat/register_node')

		try:
			self.set_state = rospy.ServiceProxy('/heartbeat/set_state', SetState)
			print "Service set_state call"
		except rospy.ServiceException, e:
			print "Service call failed: %s,"%e	

	def start(self, name):
		self.node_name = name
		if (self.register_node(self.node_name, 1)):
			print "Node heartbeat sucessfully registered"
		else:
			print "Node heartbeat failed to register"

	def stop(self, name):	
		if (not(self.unregister_node(name))):
			print "Heartbeat unregister RPC failed"		
		else:
			print "Node unregistered from Heartbeat"

	def alive(self, name):
		self.pub.publish(name)
	
	def set_node_state(self, st):
		st_from = State()
		st_to = State()
		st_from.value =  self.state
		st_to.value =  st
		resp = self.set_state(st_from,st_to,self.node_name)
		self.state = resp.current.value
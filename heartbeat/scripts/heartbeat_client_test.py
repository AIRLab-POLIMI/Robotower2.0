#!/usr/bin/env python
import rospy
from HeartbeatClientPython import HeartbeatClientPy

hb = HeartbeatClientPy()

def myhook():
	print "shutdown time!"
	hb.stop('test')

def main():
	global hb
	rospy.on_shutdown(myhook)
	rospy.init_node('test')
	r = rospy.Rate(10)

	hb.start('test')
	hb.set_node_state(1)
	rospy.spin()  	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: pass
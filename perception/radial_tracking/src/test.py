#!/usr/bin/env python
import rospy
import random
from radial_tracking.msg import SectorProbabilities

def talker():
    rospy.init_node('range_data')
    pub = rospy.Publisher('sector_probabilities', SectorProbabilities, queue_size=1)
    while not rospy.is_shutdown():
        msg = SectorProbabilities()
        msg.probabilities = [0.33,0.0,0,0,0,0,0,1]
        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
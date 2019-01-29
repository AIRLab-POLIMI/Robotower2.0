#!/usr/bin/env python
import rospy
import sqlite3
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from trueskill_manager import TrueskillManager


if __name__ == '__main__':
    rospy.init_node('outcome_generator_stub', anonymous=True)
    rate = rospy.Rate(0.3) # 1hz

    outcome_pub = rospy.Publisher('/outcome', Int8, queue_size=10)
    reset_pub = rospy.Publisher('/reset', Bool, queue_size=10)
    rate.sleep()
    rospy.logwarn('RESETTING')
    reset_pub.publish(True)

    i = 0
    while not rospy.is_shutdown():
        rate.sleep()
        outcome_pub.publish(1)
        i += 1
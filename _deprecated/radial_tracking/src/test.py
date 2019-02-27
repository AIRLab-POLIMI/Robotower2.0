#!/usr/bin/env python
import rospy
import random
from radial_tracking.msg import SectorProbabilities

def talker():
    rospy.init_node('range_data')
    pub = rospy.Publisher('sector_probabilities', SectorProbabilities, queue_size=1)

    sectors_labels = [(0, "-180/135"),
                    (1, "-135/90"),
                    (2, "-90/45"),
                    (3, "-45/0"),
                    (4, "0/45"),
                    (5, "45/90"),
                    (6, "90/135"),
                    (7, "135/180")]

    plot_ordering = [4, 5, 6, 7, 0, 1, 2, 3]

    while not rospy.is_shutdown():
        msg = SectorProbabilities()
        msg.probabilities = [0.33,0.0,0,0,0,0,0,1]
        msg.labels  = sectors_labels
        msg.plot_ordering = plot_ordering
        msg.on_dead_rck = False
        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
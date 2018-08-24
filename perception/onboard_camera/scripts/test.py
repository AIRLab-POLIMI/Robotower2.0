#!/usr/bin/env python
import rospy, tf
import random
from radial_tracking.msg import SectorProbabilities

listener = tf.TransformListener()
laser_pts = {"xs": [], "ys": []}

def legCallback(msg):
    global laser_pts

    for leg in msg.legs:
      try:
        listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
        laser_point=PointStamped()
        laser_point.header.frame_id = "base_link"
        laser_point.header.stamp = msg.header.stamp
        p = listener.transformPoint("base_link",laser_point)
        laser_pts["xs"].append(p.x)
        laser_pts["ys"].append(p.y)
      except Exception as e:
        rospy.logerr("Error in log callback!")

def reset():
    global laser_pts
    laser_pts = {"xs": [], "ys": []}

def talker():
    rospy.init_node('range_data')
    pub = rospy.Publisher('sector_probabilities', SectorProbabilities, queue_size=1)

    sectors_labels = [str(i) for i in range(0, 359, 30)]
    plot_ordering = list(range(12))

    while not rospy.is_shutdown():
        msg = SectorProbabilities()
        msg.probabilities = [0,0,0,0,0,0,0,1,0,0,0,0]
        msg.labels  = sectors_labels
        msg.plot_ordering = plot_ordering
        msg.on_dead_rck = False
        pub.publish(msg)
        reset()
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
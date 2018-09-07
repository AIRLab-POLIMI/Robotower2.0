#!/usr/bin/env python2
import rospy
import tf

# Custom messages
from leg_tracker.msg import Person, PersonArray, Leg, LegArray 
from visualization_msgs.msg import Marker

from geometry_msgs.msg import PoseWithCovarianceStamped, PolygonStamped, Point32

import collections
import numpy as np

class ExtractorBoundingBox:
    '''Uses initialpose from rviz to draw a bounding box. Use the points to manually set
    the x-y-min-max on extract_positive_training_clusters launch file.'''   
    def __init__(self):

        self.points = collections.deque(maxlen=2)
        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('distance_Legs', Marker, queue_size=300)

        #self.f = open("/home/airlab/Desktop/file.txt", "w")

        # ROS subscribers         
        self.sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initialPoseCallback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', PolygonStamped, queue_size=1)

        #rospy.spin() # So the node doesn't immediately shut down

    
    def initialPoseCallback(self, msg):
        self.points.append(msg)
        rospy.logwarn("#points in buffer: {}/2 (drops as in FIFO)".format(len(self.points)))

        if len(self.points) == 2:
            rospy.loginfo("p1: ({},{}) \t p2: ({},{})".format(self.points[0].pose.pose.position.x,self.points[0].pose.pose.position.y,
            self.points[1].pose.pose.position.x,self.points[1].pose.pose.position.y))

    def publish(self):
        if len(self.points) == 2:
            
            polygon = PolygonStamped()
            polygon.header.stamp = rospy.get_rostime()
            polygon.header.frame_id = 'base_link'
            p1 = Point32();
            p2 = Point32();
            p1.x = self.points[0].pose.pose.position.x
            p1.y = self.points[0].pose.pose.position.y
            p2.x = self.points[1].pose.pose.position.x
            p2.y = self.points[1].pose.pose.position.y

            polygon.polygon.points.append(p1)
            p11 = Point32();
            p11.x = p1.x
            p11.y = p1.y + (p2.y - p1.y)
            p12 = Point32();
            p12.x = p1.x + (p2.x - p1.x)
            p12.y = p1.y
            polygon.polygon.points.append(p1)
            polygon.polygon.points.append(p11)
            polygon.polygon.points.append(p2)
            polygon.polygon.points.append(p12)
            polygon.polygon.points.append(p1)

            self.pub.publish(polygon)



if __name__ == '__main__':
    rospy.init_node('leg_distance_extractor_bounding_box', anonymous=True)
    bd = ExtractorBoundingBox()

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        bd.publish()
        r.sleep()
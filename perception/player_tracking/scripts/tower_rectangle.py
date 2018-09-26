#!/usr/bin/python

"""Filter out legs from know rectangle (playground area)"""

import rospy
import itertools
import tf

import numpy as np

from visualization_msgs.msg import Marker
from player_tracker.msg import LegArray
from scipy import spatial

from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, Point, Pose


class Filter(object):

    def __init__(self):
        # ROS leg subscriber
        self.sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.callback)
        self.pub = rospy.Publisher('filtered_detected_leg_clusters', LegArray, queue_size=5)
        self.pub_marker = rospy.Publisher('filtered_detected_leg_clusters_marker', Marker, queue_size = 10)
        
        self.tf_listener = tf.TransformListener()

        self.tower_positions, _ = self.get_tower_distances()

        self.tower_triangle_areas = []
        self.tower_target_distances = set([])
        
        for perm in itertools.permutations(self.tower_positions, r=3):
            dist1_2 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[1].x, perm[1].y]))
            dist1_3 = spatial.distance.euclidean(np.array([perm[0].x, perm[0].y]),np.array([perm[2].x, perm[2].y]))
            dist2_3 = spatial.distance.euclidean(np.array([perm[1].x, perm[1].y]),np.array([perm[2].x, perm[2].y]))
            self.tower_target_distances.add(dist1_2)
            self.tower_target_distances.add(dist1_3)
            self.tower_target_distances.add(dist2_3)
            p = (dist1_2 + dist1_3 + dist2_3) / 2.0     # perimeter
            area = np.sqrt(p*(p-dist1_2)*(p-dist1_3)*(p-dist2_3))
            self.tower_triangle_areas.append(area)
        
        self.tower_triangle_areas = list(set(self.tower_triangle_areas))
        self.tower_target_distances = list(set(self.tower_target_distances))  


    def callback(self, msg):
        '''Manages incoming leg detections'''
        self.get_vertices(msg)


    def get_trans_wrt_robot(self, tower):
        """
        Gets tower position with respect to base_link. That is, performs a TF transformation from 'tower_link' to /base_link and returns
        x,y and theta.
        Param:
            @tower the name of the tower tf.
        Returns:
            @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform('/base_link', tower, rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform('/base_link', tower, rospy.Time(0))
            # transform from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
 
            return np.array([trans[0], trans[1], euler[2]])   # [xR,yR,theta]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))

    def get_tower_distances(self, num_towers=4):
        """Calculate tower distances from each other in the robot frame"""
        # TODO: get num of towers from param server
        towers = []
        for t in range(1,num_towers+1):
            tower = self.get_trans_wrt_robot("/tower_"+str(t))
            towers.append(Point(tower[0],tower[1],0))

        distances = []
        for t in range(num_towers):
            if t != num_towers-1:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y) , (towers[t+1].x,towers[t+1].y)),3))
            else:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y), (towers[0].x,towers[0].y)),3))
        
        return towers, distances

    def is_close_enough(self, num1, num2, tol=0.1):
        '''Checks whether two numbers are close enough from each other.'''
        return abs(num1-num2) < tol

    def is_compatible_with_playground(self, side_to_compare):
        '''Checks whether side is compaticle with know distances calculated 
        from initial (target) playground'''
        
        for d in self.tower_target_distances:
            if(self.is_close_enough(side_to_compare, d, tol=0.1)):
                return True
        return False

    
    def publish_marker(self, id, position):
        '''publish marker for the legs'''

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.id = id
        marker.type = marker.POINTS
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = position
        marker.lifetime = rospy.Duration(0.5)
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color.a = 1.0
        marker.color.r = 1.0

        self.pub_marker.publish(marker)

    def get_vertices(self, detected_clusters_msg):
        '''Get playground rectangle vertices, aka Towers.'''

        estimated_towers = []

        if len(detected_clusters_msg.legs) > 3:
            for perm in itertools.permutations(detected_clusters_msg.legs, r=3):
                dist1_2 = spatial.distance.euclidean(np.array([perm[0].position.x, perm[0].position.y]),np.array([perm[1].position.x, perm[1].position.y]))
                dist1_3 = spatial.distance.euclidean(np.array([perm[0].position.x, perm[0].position.y]),np.array([perm[2].position.x, perm[2].position.y]))
                dist2_3 = spatial.distance.euclidean(np.array([perm[1].position.x, perm[1].position.y]),np.array([perm[2].position.x, perm[2].position.y]))
                triangle_distances = [dist1_2, dist1_3, dist2_3]
                p = (dist1_2 + dist1_3 + dist2_3) / 2.0     # perimeter
                
                area = np.sqrt(p*(p-dist1_2)*(p-dist1_3)*(p-dist2_3))
                
                for a in self.tower_triangle_areas:
                    if self.is_close_enough(a, area, tol=0.1):
                        success = True

                        for side in triangle_distances:
                            if not self.is_compatible_with_playground(side):
                                success = False
                                break

                        if success:
                            estimated_towers = [p.position for p in perm]
        
        # loop through estimated towers
        if len(estimated_towers) != 0:
            to_exclude = []

            # get indexes of estimated towers on the original LegArray msg.
            for idx, leg in enumerate(detected_clusters_msg.legs):
                for t in estimated_towers:
                    if (leg.position.x == t.x and 
                        leg.position.y == t.y):
                        to_exclude.append(idx)
            
            # exclude indexes found
            detected_clusters_msg.legs = [detected_clusters_msg.legs[idx] for idx in range(len(detected_clusters_msg.legs))
                                            if idx not in to_exclude]
            # for idx in to_exclude:
            #     rospy.logwarn(idx)
            #     detected_clusters_msg.legs.pop(idx)

            # publish remaining legs in case they exist
            if len(detected_clusters_msg.legs) != 0:
                self.pub.publish(detected_clusters_msg)
            
            # publish markers
            for idx, i in enumerate(detected_clusters_msg.legs):
                self.publish_marker(idx, i)

if __name__ == "__main__":
    rospy.init_node('tower_leg_filter', anonymous=True)
    f = Filter()
    rospy.spin()
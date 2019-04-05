#!/usr/bin/env python2
import rospy
import tf
import math
import copy
import numpy as np
import itertools 

from scipy import spatial

# Custom messages
from player_tracker.msg import Person, PersonArray, Leg, LegArray, PersonEvidence, PersonEvidenceArray, TowerArray, Tower
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped, Point, Pose


class TowerRectCreator(object): 

    '''Calculates the distance between LEG clusters and saves to file.''' 
    
    def __init__(self):

        self.tower_triangle_areas = []
        self.tf_listener = tf.TransformListener()
        self.pub_tower_markers = rospy.Publisher('tower_rectangle', PolygonStamped, queue_size=1)
        self.pub_towers = rospy.Publisher('estimated_tower_positions', TowerArray, queue_size=5)
        self.fixed_frame = rospy.get_param('fixed_frame')

        self.bound_box_points = []

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', PolygonStamped, queue_size=1)
        self.tower_markers_pub = rospy.Publisher('tower_markers', MarkerArray, queue_size=1)

        self.tower_positions, _ = self.get_tower_distances()
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

        #rospy.spin() # So the node doesn't immediately shut down

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

    def is_compatible_with_playground(self, side_to_compare):
        for d in self.tower_target_distances:
            if(self.is_close_enough(side_to_compare, d, tol=0.1)):
                return True
        return False

    def publish_poligon(self, pts):
        '''Publish poligon for the bouding box'''
        
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'map'
        polygon.polygon.points = self.sort_vertices([p.position for p in pts])
        #polygon.polygon.points = [p.position for p in pts]
        self.bound_box_points = polygon.polygon.points
        self.pub_tower_markers.publish(polygon)
        #self.publish()

    def get_tower_distances(self, num_towers=4):
        """Calculate tower distances from each other in the robot frame"""
        # TODO: get num of towers from param server
        towers = []
        for t in range(1,num_towers+1):
            tower = self.get_trans_wrt_robot("tower_"+str(t))
            towers.append(Point(tower[0],tower[1],0))

        distances = []
        for t in range(num_towers):
            if t != num_towers-1:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y) , (towers[t+1].x,towers[t+1].y)),3))
            else:
                distances.append(round(spatial.distance.euclidean((towers[t].x,towers[t].y), (towers[0].x,towers[0].y)),3))
        
        return towers, distances

    def publish(self):
        '''Publish poligon for the bouding box'''
            
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'map'
        polygon.polygon.points = self.bound_box_points #self.sort_vertices()
        self.pub.publish(polygon)

    def sort_vertices(self, pts):
        # Sorts vertices of polygon to get a rectangle
        if(len(pts) == 0):
            return []   
        if(len(self.bound_box_points) == 0):
            self.bound_box_points = [None] * 4
                 
        pts.sort(self.point_x_comparator)
        #rospy.loginfo(pts)
        self.bound_box_points[0] = pts[0]
        self.bound_box_points[1] = pts[2]
        self.bound_box_points[2] = pts[3]
        self.bound_box_points[3] = pts[1]

        return self.bound_box_points

    def point_x_comparator(self, p1, p2):
        # Comparator to sort points wrt x-axis
        if(p1.x > p2.x):
            return 1
        else:
            return -1

    def inside_polygon(self, x, y):
        """
        Return True if a coordinate (x, y) is inside a polygon defined by
        a list of verticies [(x1, y1), (x2, x2), ... , (xN, yN)].
        Reference: http://www.ariel.com.au/a/python-point-int-poly.html
        """
        points = self.bound_box_points

        n = len(points)
        inside = False
        p1x = points[0].x
        p1y = points[0].y
        for i in range(1, n + 1):
            p2x, p2y = points[i % n].x, points[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def is_close_enough(self, num1, num2, tol=0.1):
        return abs(num1-num2) < tol

    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """
        self.search_bounding_box(detected_clusters_msg)
    
    def publish_tower_markers(self, points):
        tower_markers = MarkerArray()
        i = 0
        for point in points:
            m = Marker()
            m.id = i
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = '/map'
            m.type = Marker.SPHERE
            m.pose.position.x = point.position.x
            m.pose.position.y = point.position.y

            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15 

            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0

            tower_markers.markers.append(m)
            i += 1
        self.tower_markers_pub.publish(tower_markers)
    

    def search_bounding_box(self, detected_clusters_msg):
        tower_array_msg = TowerArray()
        tower_array_msg.header = detected_clusters_msg.header
        if len(detected_clusters_msg.legs) > 3:
            #rospy.logerr("Checking every possible triangle")
            for perm in itertools.permutations(detected_clusters_msg.legs, r=3):
                dist1_2 = spatial.distance.euclidean(np.array([perm[0].position.x, perm[0].position.y]),np.array([perm[1].position.x, perm[1].position.y]))
                dist1_3 = spatial.distance.euclidean(np.array([perm[0].position.x, perm[0].position.y]),np.array([perm[2].position.x, perm[2].position.y]))
                dist2_3 = spatial.distance.euclidean(np.array([perm[1].position.x, perm[1].position.y]),np.array([perm[2].position.x, perm[2].position.y]))
                triangle_distances = [dist1_2, dist1_3, dist2_3]
                p = (dist1_2 + dist1_3 + dist2_3) / 2.0     # perimeter
                
                area = np.sqrt(p*(p-dist1_2)*(p-dist1_3)*(p-dist2_3))
                
                for a in self.tower_triangle_areas:
                    if self.is_close_enough(a, area, tol=0.05):
                        to_pub = True

                        for side in triangle_distances:
                            if not self.is_compatible_with_playground(side):
                                to_pub = False
                                break

                        if to_pub:
                            tower_array_msg.towers = [Tower(p.position, p.points, p.point_indexes) for p in perm]
                            missing_vertex = self.get_missing_vertex(tower_array_msg.towers)
                            # missing_cluster = self.get_missing_cluster(missing_vertex, detected_clusters_msg)
                            # if(missing_cluster == None):
                            tower_array_msg.towers.append(Tower(missing_vertex.position, [], [])) 
                            # else:
                                # print "Found missing Cluster"
                                # rospy.loginfo("Cluster position: {}".format(missing_cluster.position))
                                # tower_array_msg.towers.append(Tower(missing_cluster.position, missing_cluster.points, missing_cluster.point_indexes)) 
                            vertices = list(perm)
                            vertices.append(missing_vertex)
                            self.publish_poligon(vertices)
                            self.publish_tower_markers(vertices)
                            
        if len(tower_array_msg.towers) != 0:
            self.pub_towers.publish(tower_array_msg)

    def get_missing_vertex(self, towers):
        sides = []
        sides.append( (towers[0], towers[1]))
        sides.append( (towers[1], towers[2]))
        sides.append( (towers[2], towers[0]))

        sides.sort(self.my_comparator, reverse=True) # Sort sides from longest (hypotenuse) to shortest (catethus min)

        vertex_to_extend = sides[0][0] # Take one vertex of the hypotenuse

        if(self.contains_vertex(sides[1], vertex_to_extend)): #Identify which is the opposite cathetus wrt the selected vertex
            opposite_cathetus = 2
        else:
            opposite_cathetus = 1

        for i in range( len(sides[opposite_cathetus]) ):
            if( self.contains_vertex(sides[0], sides[opposite_cathetus][i]) == False ):
                #Identify the vertex where the catheti join, the one with the rect angle             
                rect_vertex_index = i 
            else:
                not_rect_vertex_index = i

        
        # Calculate the orientation of the opposite cathetus centered in the vertex where the two catheti join
        extension_angle = np.arctan2(sides[opposite_cathetus][rect_vertex_index].position.y - sides[opposite_cathetus][not_rect_vertex_index].position.y,
            sides[opposite_cathetus][rect_vertex_index].position.x - sides[opposite_cathetus][not_rect_vertex_index].position.x)

        # Calculate the lenght of the opposite cathetus
        extension_magnitude = ( (sides[opposite_cathetus][0].position.x - sides[opposite_cathetus][1].position.x)**2 + 
            (sides[opposite_cathetus][0].position.y - sides[opposite_cathetus][1].position.y)**2 )**0.5

        # Calculate the cartesian projections of the opposite cathetus
        extension_x = extension_magnitude * np.cos(extension_angle + np.pi)
        extension_y = extension_magnitude * np.sin(extension_angle + np.pi)

        
        # Reconstruct missing vertex by extending the chosen one from the hypotenuse
        missing_vertex = Pose()
        missing_vertex.position.x = vertex_to_extend.position.x + extension_x
        missing_vertex.position.y = vertex_to_extend.position.y + extension_y
        missing_vertex.position.z = 0

        return missing_vertex

    def get_missing_cluster(self, missing_vertex, detected_clusters_msg):
        ''' Returns the closest cluster to the estimated missing vertex'''
        # Convert missing vertex from map frame to robot frame 
        point_to_transform = PointStamped()
        point_to_transform.header.stamp = rospy.Time()
        point_to_transform.header.frame_id = "/map"

        point_transformed = self.tf_listener.transformPoint(point_to_transform)
        
        for cluster in detected_clusters_msg.legs:
            centroid = cluster.position
            if self.is_close_enough(missing_vertex.position.x, centroid.x) and self.is_close_enough(missing_vertex.position.y, centroid.y):
                return cluster

        return None

    def my_comparator(self, side_a, side_b):
        # Returns the longest side
        length_a = (side_a[0].position.x - side_a[1].position.x)**2 + (side_a[0].position.y - side_a[1].position.y)**2
        length_b = (side_b[0].position.x - side_b[1].position.x)**2 + (side_b[0].position.y - side_b[1].position.y)**2

        if(length_a > length_b):
            return 1
        return -1

    def are_pose_equal(self, pose_1, pose_2, thd = 0.1):
        if( abs(pose_1.position.x - pose_2.position.x) < thd and abs(pose_1.position.y - pose_2.position.y) < thd) :
            return True
        return False

    def contains_vertex(self, side, vertex):
        if(self.are_pose_equal(side[0], vertex) or self.are_pose_equal(side[1], vertex)):
            return True
        #if(side[0] == vertex or side[1] == vertex):
            #return True
        return False

    def match_cluster(self, vertex, accepted_clusters, acceptance_thd = 0.1):
        """ Trying to match the given vertex with one of the accepted_clusters """
        for cluster in accepted_clusters:
            centroid = cluster.position
            if( abs(vertex.position.x - centroid.x) < acceptance_thd and abs(vertex.position.y - centroid.y) < acceptance_thd ):
                return cluster
        return None

if __name__ == '__main__':
    rospy.init_node('tower_rectangle_creator')
    creator = TowerRectCreator()
    rospy.spin()

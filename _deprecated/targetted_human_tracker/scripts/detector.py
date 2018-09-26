#!/usr/bin/python

import rospy

# Custom messages
from targetted_human_tracker.msg import Person, PersonArray, Leg, LegArray, PersonEvidenceArray

# ROS messages
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# Standard python modules
import numpy as np
import random
import math
import scipy.stats
from scipy import spatial
import tf
import copy
import timeit
import message_filters
import sys
import tf
import cv2

from itertools import groupby
from operator import itemgetter

from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import PointCloud2

from perception_grid_match import PerceptionGridProjector
from sensor_msgs import point_cloud2
from laser_geometry import LaserProjection

# External modules
from munkres import Munkres # For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
from pykalman import KalmanFilter # To install: http://pykalman.github.io/#installation
from auxiliar import KinectEvidence, PolarGrid

class Position(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def __str__(self):
        return "({:.2f},{:.2f})".format(self.x, self.y)

    def euclideanDistance(self,other):
        return np.sqrt((other.x - self.x)**2 + (other.y - self.y)**2)
    
    def __eq__(self, other):
        """Overrides the default implementation"""
        if isinstance(self, other.__class__):
            return self.x == other.x and self.y == other.y
        return False

    def __ne__(self, other):
        """Overrides the default implementation (unnecessary in Python 3)"""
        return not self.__eq__(other)
    
    def __add__(self, other):
        return Position(self.x + other.x, self.y + other.y)

    def to_list(self):
        return [self.x,self.y]

class Observation(object):
    '''A person evidence. Not yet associated to an existing track.'''
    def __init__(self, pos_x, pos_y, confidence, in_free_space):
        self.position = Position(pos_x, pos_y)
        self.confidence = confidence
        self.in_free_space = in_free_space
        self.in_free_space_bool = None

class ObjectTracked:
    """
    Used for maintaining object.
    """
    id = 0

    @staticmethod
    def getId():
        '''Return new Id'''
        ObjectTracked.id +=1
        return ObjectTracked.id

    def __init__(self, x, y, now, confidence, is_player, in_free_space):  
        self.id_num = ObjectTracked.getId()
        self.is_player = is_player
        self.colour = (0,0,0)
        self.setIsPlayer(is_player)
        self.last_seen = now
        self.seen_in_current_scan = True
        self.isPlayer = is_player
        self.times_seen = 1
        self.confidence = confidence
        self.dist_travelled = 0.
        self.deleted = False
        self.in_free_space = in_free_space

        # People are tracked via a constant-velocity Kalman filter with a Gaussian acceleration distribution
        # Kalman filter params were found by hand-tuning. 
        # A better method would be to use data-driven EM find the params. 
        # The important part is that the observations are "weighted" higher than the motion model 
        # because they're more trustworthy and the motion model kinda sucks
        scan_frequency = rospy.get_param("scan_frequency", 7.5)
        delta_t = 1./scan_frequency
        if scan_frequency > 7.49 and scan_frequency < 7.51:
            std_process_noise = 0.06666
        elif scan_frequency > 9.99 and scan_frequency < 10.01:
            std_process_noise = 0.05
        elif scan_frequency > 14.99 and scan_frequency < 15.01:
            std_process_noise = 0.03333
        else:
            print "Scan frequency needs to be either 7.5, 10 or 15 or the standard deviation of the process noise needs to be tuned to your scanner frequency"
        
        std_pos = std_process_noise
        std_vel = std_process_noise
        std_obs = 0.1
        var_pos = std_pos**2
        var_vel = std_vel**2
        
        # The observation noise is assumed to be different when updating the Kalman filter than when doing data association
        var_obs_local = std_obs**2 
        self.var_obs = (std_obs + 0.4)**2

        self.filtered_state_means = np.array([x, y, 0, 0])
        self.pos_x = x
        self.pos_y = y
        self.vel_x = 0
        self.vel_y = 0

        self.filtered_state_covariances = 0.5*np.eye(4) 

        # Constant velocity motion model
        transition_matrix = np.array([[1, 0, delta_t,        0],
                                      [0, 1,       0,  delta_t],
                                      [0, 0,       1,        0],
                                      [0, 0,       0,        1]])

        # Observation model. Can observe pos_x and pos_y (unless person is occluded). 
        observation_matrix = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0]])

        transition_covariance = np.array([[var_pos,       0,       0,       0],
                                          [      0, var_pos,       0,       0],
                                          [      0,       0, var_vel,       0],
                                          [      0,       0,       0, var_vel]])

        observation_covariance =  var_obs_local*np.eye(2)

        self.kf = KalmanFilter(
            transition_matrices=transition_matrix,
            observation_matrices=observation_matrix,
            transition_covariance=transition_covariance,
            observation_covariance=observation_covariance,
        )


    def update(self, observations):
        """
        Update our tracked object with new observations
        """
        self.filtered_state_means, self.filtered_state_covariances = (
            self.kf.filter_update(
                self.filtered_state_means,
                self.filtered_state_covariances,
                observations
            )
        )

        # Keep track of the distance it's travelled 
        # We include an "if" structure to exclude small distance changes, 
        # which are likely to have been caused by changes in observation angle
        # or other similar factors, and not due to the object actually moving
        delta_dist_travelled = ((self.pos_x - self.filtered_state_means[0])**2 + (self.pos_y - self.filtered_state_means[1])**2)**(1./2.) 
        if delta_dist_travelled > 0.01: 
            self.dist_travelled += delta_dist_travelled

        self.pos_x = self.filtered_state_means[0]
        self.pos_y = self.filtered_state_means[1]
        self.vel_x = self.filtered_state_means[2]
        self.vel_y = self.filtered_state_means[3]

    def getPosition(self):
        '''Return the object x,y position from Kalman'''
        return Position(self.pos_x,self.pos_y)

    def setIsPlayer(self, is_player):
        self.is_player = is_player
        if self.is_player:
            self.colour = (0.9,0.05,0.05)
        else:
            self.colour = (0.2,0.2,0.2)


class Tracker:    
    """
    Tracker for tracking all the people and objects
    """
    max_cost = float('inf')

    def __init__(self):
        self.player_trak = [] 
        self.objects_tracked = []
        self.potential_leg_pairs = set()
        self.potential_leg_pair_initial_dist_travelled = {}
        self.people_tracked = []
        self.prev_track_marker_id = 0
        self.prev_person_marker_id = 0
        self.prev_time = None
        self.listener = tf.TransformListener()
        self.local_map = None
        self.new_local_map_received = True

        self.lda_blob = 0.5
        self.lda_player = 0.5
        self.player_position = None

        random.seed(1) 

        # Get ROS params
        self.fixed_frame = rospy.get_param("fixed_frame", "odom")
        self.max_leg_pairing_dist = rospy.get_param("max_leg_pairing_dist", 0.8)
        self.confidence_threshold_to_maintain_track = rospy.get_param("confidence_threshold_to_maintain_track", 0.5)
        self.publish_occluded = rospy.get_param("publish_occluded", True)
        self.publish_people_frame = rospy.get_param("publish_people_frame", self.fixed_frame)
        self.use_scan_header_stamp_for_tfs = rospy.get_param("use_scan_header_stamp_for_tfs", False)
        self.publish_detected_people = rospy.get_param("display_detected_people", False)        
        self.dist_travelled_together_to_initiate_leg_pair = rospy.get_param("dist_travelled_together_to_initiate_leg_pair", 0.5)
        scan_topic = rospy.get_param("scan_topic", "scan")
        self.scan_frequency = rospy.get_param("scan_frequency", 7.5)
        self.in_free_space_threshold = rospy.get_param("in_free_space_threshold", 0.06)
        self.confidence_percentile = rospy.get_param("confidence_percentile", 0.90)
        self.max_std = rospy.get_param("max_std", 1)

        self.mahalanobis_dist_gate = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, 1.0)
        self.max_cov = self.max_std**2
        self.latest_scan_header_stamp_with_tf_available = rospy.get_rostime()

    	# ROS publishers
        self.people_tracked_pub = rospy.Publisher('players_tracked', PersonArray, queue_size=300)
        self.people_detected_pub = rospy.Publisher('players_detected', PersonArray, queue_size=300)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=300)
        self.non_leg_clusters_pub = rospy.Publisher('non_leg_clusters', LegArray, queue_size=300)
        #self.laser_cloud_pub = rospy.Publisher('laser_cloud', PointCloud2, queue_size=1)

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('person_evidence_array', PersonEvidenceArray, self.evidenceCallback)
        #self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      
        self.local_map_sub = rospy.Subscriber('local_map', OccupancyGrid, self.local_map_callback)
        #self.sub_map = rospy.Subscriber('map', OccupancyGrid, self.map_callback)  # create map subscriber
        #self.sub_local_grid = rospy.Subscriber('test_local_map', OccupancyGrid, self.local_grid_callback)  # create map subscriber
        #self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.laser_callback)  # create laser subscriber

        # Polar grid
        self.polar_grid = PolarGrid()

        self.tf_listener = tf.TransformListener()

        self.map_kp = None
        self.map_desc  = None

        self.perc_kp = None
        self.perc_desc = None

        self.laser_projection = LaserProjection()
        self.perception_grid_projector = PerceptionGridProjector()

        rospy.spin() # So the node doesn't immediately shut down

    def local_grid_callback(self,msg):
        if self.map_kp is not None and self.map_desc is not None:
            image = np.array(copy.deepcopy(msg.data)).reshape(msg.info.width,msg.info.height)
            # fix range of map
            image = 255 - image * (255.0/float(image.max()))

            image=image.astype(np.uint8)
            #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            #cv2.imshow('image',image)
            #cv2.imwrite('/home/airlab/Scrivania/image_grid.png',image)

            # map descriptor
            self.perc_kp, self.perc_desc = self.perception_grid_projector.compute_descriptors(image)

            rotation_mtx = self.perception_grid_projector.get_rotation_matrix(self.map_kp, self.map_desc, self.perc_kp, self.perc_desc)
            trans_ = self.perception_grid_projector.project_image(rotation_mtx, image)
            print trans_
            cv2.imwrite('/home/airlab/Scrivania/trans_.png',trans_)

    def map_callback(self, msg):
        image = np.array(copy.deepcopy(msg.data)).reshape(msg.info.width,msg.info.height)

        # fix range of map
        image = 255 - image * (255.0/float(image.max()))

        image=image.astype(np.uint8)
        
        #cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        #cv2.imshow('image',image)
        #cv2.imwrite('/home/airlab/Scrivania/image.png',image)

        # map descriptor
        self.map_kp, self.map_desc = self.perception_grid_projector.compute_descriptors(image)
        
        # unsubscribe to topic map. We need it only once.
        self.sub_map.unregister()
        rospy.logwarn("Unsubscribed to map")

    def local_map_callback(self, map):
        """
        Local map callback to update our local map with a newly published one
        """
        self.local_map = map
        self.new_local_map_received = True


    def how_much_in_free_space(self, evidence):
        """
        Determine the degree to which the position (x,y) is in freespace according to our local map

        @rtype:     float
        @return:    degree to which the position (x,y) is in freespace (range: 0.-1.)
        """

        x = None
        y = None 

        if isinstance(evidence, Observation):
            x = evidence.position.x
            y = evidence.position.y
        elif isinstance(evidence, Position):
            x = evidence.x
            y = evidence.y
        else:
            x = (evidence.leg1.x + evidence.leg2.x)/2.
            y = (evidence.leg1.y + evidence.leg2.y)/2.

        # If we haven't got the local map yet, assume nothing's in freespace
        if self.local_map == None:
            return self.in_free_space_threshold*2

        # Get the position of (x,y) in local map coords
        map_x = int(round((x - self.local_map.info.origin.position.x)/self.local_map.info.resolution))
        map_y = int(round((y - self.local_map.info.origin.position.y)/self.local_map.info.resolution))

        # Take the average of the local map's values centred at (map_x, map_y), with a kernel size of <kernel_size>
        # If called repeatedly on the same local_map, this could be sped up with a sum-table
        sum = 0
        kernel_size = 2
        for i in xrange(map_x-kernel_size, map_x+kernel_size):
            for j in xrange(map_y-kernel_size, map_y+kernel_size):
                if i + j*self.local_map.info.height < len(self.local_map.data):
                    sum += self.local_map.data[i + j*self.local_map.info.height]
                else:  
                    # We went off the map! position must be really close to an edge of local_map
                    return self.in_free_space_threshold*2

        percent = sum/(((2.*kernel_size + 1)**2.)*100.)
        return percent

    def getPlayerKinectPosition(self):
        """
        Gets the player blob position.
        """
        try:
            self.tf_listener.waitForTransform('/player_filtered_link','/base_link', rospy.Time(0), rospy.Duration(0.1))
            trans, rot = self.tf_listener.lookupTransform('/player_filtered_link','/base_link', rospy.Time(0))
 
            return Position(trans[0], trans[1])

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Detector node: " + str(e))

        
    def match_detections_to_tracks_GNN(self, updated_tracks, observations):
        """
        Match detected objects to existing object tracks using a global nearest neighbour data association
        """
        
        matched_tracks = {}
        blob_distances = {}

        blob_position = self.getPlayerKinectPosition()

        for track in updated_tracks:
            o = '_'
            mdistance = float('inf')
            for obs in observations:   
                dist = obs.position.euclideanDistance(track.getPosition())
                if (dist < mdistance) and obs.confidence > 0.5:
                    mdistance = dist
                    o = obs
            matched_tracks[track] = o if mdistance < 1.5 else None
            blob_distances[track] = obs.position.euclideanDistance(blob_position)

        return matched_tracks, blob_distances

    def evidenceCallback(self, evidences):

        now = evidences.header.stamp

        if len(self.objects_tracked) == 0:
            for evidence in evidences.evidences:
                p = Position((evidence.leg1.x + evidence.leg2.x)/2.,(evidence.leg1.y + evidence.leg2.y)/2.)
                new_observation = ObjectTracked((evidence.leg1.x + evidence.leg2.x)/2., 
                                                (evidence.leg1.y + evidence.leg2.y)/2., 
                                                now, 
                                                evidence.probability + self.RBFKernel(p,self.getPlayerKinectPosition(),self.lda_blob),
                                                False, 
                                                in_free_space=self.how_much_in_free_space(evidence))
                self.objects_tracked.append(new_observation)

            blob_pos = self.getPlayerKinectPosition()                ## add blob position as evidence as well.
            if self.player_position is None:
                new_observation = ObjectTracked(blob_pos.x,blob_pos.y,
                                            now,
                                            1,
                                            False,
                                            in_free_space=self.how_much_in_free_space(blob_pos))
            else:
                new_observation = ObjectTracked(blob_pos.x,blob_pos.y,
                                            now,
                                            self.calcNewConfidence(blob_pos),
                                            False,
                                            in_free_space=self.how_much_in_free_space(blob_pos))  
            self.objects_tracked.append(new_observation)
            return
        else:
            ### define the observations based on their position in free-space ###
            observations= []
            for evidence in evidences.evidences:
                new_observation = Observation((evidence.leg1.x + evidence.leg2.x)/2.,
                                              (evidence.leg1.y + evidence.leg2.y)/2.,
                                              evidence.probability,
                                              in_free_space=self.how_much_in_free_space(evidence))  

                if new_observation.in_free_space < self.in_free_space_threshold:
                    new_observation.in_free_space_bool = True
                else:
                    new_observation.in_free_space_bool = False
                observations.append(new_observation)


            ### ------------------------------------------------------------- ###

            ### Propogate existing tracks ###
            updated_tracks = []
            for propogated_track in self.objects_tracked:
                propogated_track.update(np.ma.masked_array(np.array([0, 0]), mask=[1,1])) 
                updated_tracks.append(propogated_track)
            ### ------------------------- ###
            
            # Match detected objects to existing tracks
            matched_tracks, blob_distances = self.match_detections_to_tracks_GNN(updated_tracks, observations)

            ### Update all tracks with new observations ###
            tracks_to_delete = set()   
            for idx, track in enumerate(self.objects_tracked):
                propagated_track = updated_tracks[idx]          #  Get the corresponding propogated track
            
                if matched_tracks[track] is None:
                    # propogated_track not matched to a detection
                    # don't provide a measurement update for Kalman filter 
                    # so send it a masked_array for its observations
                    obs = np.ma.masked_array(np.array([0, 0]), mask=[1,1]) 
                    #track.kf.observation_covariance = np.eye(2)
                    track.seen_in_current_scan = False
                    # Input observations to Kalman filter
                    track.update(obs)
                
                else:
                    xy_observation = np.array([matched_tracks[track].position.x,matched_tracks[track].position.y])
                    track.kf.observation_covariance = np.exp(-matched_tracks[track].confidence) * np.eye(2)
                    track.in_free_space = 0.8*track.in_free_space + 0.2*matched_tracks[track].in_free_space 
                    track.confidence = 0.8*track.confidence + 0.2*self.calcNewConfidence(track.getPosition())
                    track.times_seen += 1
                    track.last_seen = now
                    track.isPlayer = False
                    track.seen_in_current_scan = True
                                
                    # Input observations to Kalman filter
                    track.update(xy_observation)

                # Check track for deletion           
                if track.confidence < self.confidence_threshold_to_maintain_track:
                    tracks_to_delete.add(track)
                    rospy.logdebug("deleting track due to low confidence...")
                else:
                    # Check track for deletion because covariance is too large
                    cov = track.filtered_state_covariances[0][0] + track.var_obs # cov_xx == cov_yy == cov
                    if cov > self.max_cov:
                        tracks_to_delete.add(track)
                        rospy.logdebug("deleting track due to large covariance...")
                        #rospy.loginfo("deleting because unseen for %.2f", (now - track.last_seen).to_sec())

            ###  ---------------------- ###

            # Delete tracks that have been set for deletion
            for track in tracks_to_delete:         
                self.objects_tracked.remove(track)
            
            for ev in observations:
                if ev not in matched_tracks.values() and ev.confidence > 0.5: ## because the evidence may been too far away...
                    self.objects_tracked.append(ObjectTracked((ev.position.x + ev.position.x)/2., (ev.position.y + ev.position.y)/2., 
                                                                now, self.calcNewConfidence(ev.position), False,
                                                                in_free_space=self.how_much_in_free_space(ev)))

        
        if len(self.objects_tracked) == 0:
            return

        #### Unite similar tracks that are smaller than epsilon ####
        tree = spatial.KDTree(np.array([[p.x, p.y] for p in [o.getPosition() for o in self.objects_tracked]]))
        epsilon = 0.5

        struct = {}
        for n in range(len(tree.data)):
            struct[n] = None

        for i, pt in enumerate(tree.data):
            nearest_point = tree.query(pt, k=2)
            j = nearest_point[1][1]
            distance = nearest_point[0][1]
            if distance > epsilon:
                struct[i] = i
            elif struct[j] is None and struct[i] is None:
                struct[i] = i
                struct[j] = i
            elif struct[j] is None:
                struct[j] = struct[i]
            else:
                struct[i] = struct[j]

        # getting conected components
        v = {}
        for key, value in sorted(struct.iteritems()):
            v.setdefault(value, []).append(key)
        struct = v
        
        new_object_tracks = []

        player_idx = None
        max_player_confidence = float('-inf')
        new = None

        # update tracks
        for key in struct.keys():
            pts = []
            for v in struct[key]:
                pt = Position(tree.data[v][0], tree.data[v][1])
                for i, obj in enumerate(self.objects_tracked):
                    if obj.getPosition() == pt:
                        pts.append(i)

            if len(pts) == 1:
                new = copy.deepcopy(self.objects_tracked[pts[0]])
            else:
                max_confidence = float('-inf')

                for p in pts:
                    if self.objects_tracked[p].confidence > max_confidence:
                        new = copy.deepcopy(self.objects_tracked[p])

            new_object_tracks.append(new)
            
            if new.confidence > max_player_confidence:
                    max_player_confidence = new.confidence
                    player_idx = new_object_tracks.index(new)
        
        if player_idx is not None:
            if  self.player_position is None:
                new_object_tracks[player_idx].setIsPlayer(True)
                self.player_position = copy.deepcopy(new_object_tracks[player_idx])
            else:
                if new_object_tracks[player_idx].getPosition().euclideanDistance(self.player_position.getPosition()) < 1: # TODO -< gambiarra
                    new_object_tracks[player_idx].setIsPlayer(True)
                    self.player_position = copy.deepcopy(new_object_tracks[player_idx])

        self.objects_tracked = new_object_tracks

        # Publish to rviz and /people_tracked topic.
        self.publish_tracked_people(now)


    def calcNewConfidence(self, position):
        p_wrt_blob = self.RBFKernel(position,self.getPlayerKinectPosition(), self.lda_blob)
        p_wrt_current_player = None
        if self.player_position is None:
            p_wrt_current_player = self.RBFKernel(position,Position(0.0,0.0), self.lda_player)
        else:
            p_wrt_current_player = self.RBFKernel(position,self.player_position.getPosition(), self.lda_player)
        return p_wrt_blob + p_wrt_current_player - p_wrt_blob * p_wrt_current_player


    def RBFKernel(self,x,b,lda):
        if isinstance(x, Position) or isinstance(b,Position):
            x = np.array(x.to_list())
            b = np.array(b.to_list())
        else:
            raise Exception('x and b must be Position instances')
        return np.exp(-lda*np.dot(np.transpose(np.array(x)-np.array(b)),(np.array(x)-np.array(b))))

    def publish_tracked_people(self, now):
        """
        Publish markers of tracked people to Rviz and to <people_tracked> topic
        """        
        people_tracked_msg = PersonArray()
        people_tracked_msg.header.stamp = now
        people_tracked_msg.header.frame_id = self.publish_people_frame        
        marker_id = 0

        # Make sure we can get the required transform first:
        if self.use_scan_header_stamp_for_tfs:
            tf_time = now
            try:
                self.listener.waitForTransform(self.publish_people_frame, self.fixed_frame, tf_time, rospy.Duration(1.0))
                transform_available = True
            except:
                transform_available = False
        else:
            tf_time = rospy.Time(0)
            transform_available = self.listener.canTransform(self.publish_people_frame, self.fixed_frame, tf_time)

        marker_id = 0

        if not transform_available:
            rospy.loginfo("Person tracker: tf not avaiable. Not publishing people")
        else:
            for person in self.objects_tracked:
                if not person.is_player:
                    continue

                if self.publish_occluded or person.seen_in_current_scan: # Only publish people who have been seen in current scan, unless we want to publish occluded people
                    # Get position in the <self.publish_people_frame> frame 
                    ps = PointStamped()
                    ps.header.frame_id = self.fixed_frame
                    ps.header.stamp = tf_time
                    ps.point.x = person.pos_x
                    ps.point.y = person.pos_y

                    try:
                        ps = self.listener.transformPoint(self.publish_people_frame, ps)
                    except:
                        rospy.logerr("Not publishing people due to no transform from fixed_frame-->publish_people_frame")                                                
                        continue
                    
                    # publish to people_tracked topic
                    new_person = Person() 
                    new_person.pose.position.x = ps.point.x 
                    new_person.pose.position.y = ps.point.y 
                    yaw = math.atan2(person.vel_y, person.vel_x)
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
                    new_person.pose.orientation.x = quaternion[0]
                    new_person.pose.orientation.y = quaternion[1]
                    new_person.pose.orientation.z = quaternion[2]
                    new_person.pose.orientation.w = quaternion[3] 
                    new_person.id = person.id_num 
                    new_person.speed = np.sqrt(person.vel_x**2 + person.vel_y**2)
                    people_tracked_msg.people.append(copy.deepcopy(new_person))

                    # publish rviz markers       
                    # Cylinder for body 
                    marker = Marker()
                    marker.header.frame_id = self.publish_people_frame
                    marker.header.stamp = now
                    marker.ns = "People_tracked"
                    marker.color.r = person.colour[0]
                    marker.color.g = person.colour[1]
                    marker.color.b = person.colour[2]        
                    marker.color.a = (rospy.Duration(3) - (rospy.get_rostime() - person.last_seen)).to_sec()/rospy.Duration(3).to_sec() + 0.1
                    marker.pose.position.x = ps.point.x 
                    marker.pose.position.y = ps.point.y
                    marker.id = marker_id 
                    marker_id += 1
                    marker.type = Marker.CYLINDER
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 1.2
                    marker.pose.position.z = 0.8
                    self.marker_pub.publish(marker)  

                    # Sphere for head shape                        
                    marker.type = Marker.SPHERE
                    marker.scale.x = 0.2
                    marker.scale.y = 0.2
                    marker.scale.z = 0.2                
                    marker.pose.position.z = 1.5
                    marker.id = marker_id 
                    marker_id += 1                        
                    self.marker_pub.publish(marker)     

                    # Text showing person's ID number 
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.TEXT_VIEW_FACING
                    marker.text = "C: {:.2f}".format(person.confidence) #str(person.id_num)
                    marker.scale.z = 0.2         
                    marker.pose.position.z = 1.7
                    self.marker_pub.publish(marker)

                    # Arrow pointing in direction they're facing with magnitude proportional to speed
                    marker.color.r = person.colour[0]
                    marker.color.g = person.colour[1]
                    marker.color.b = person.colour[2]          
                    marker.color.a = (rospy.Duration(3) - (rospy.get_rostime() - person.last_seen)).to_sec()/rospy.Duration(3).to_sec() + 0.1                        
                    start_point = Point()
                    end_point = Point()
                    start_point.x = marker.pose.position.x 
                    start_point.y = marker.pose.position.y 
                    end_point.x = start_point.x + 0.5*person.vel_x
                    end_point.y = start_point.y + 0.5*person.vel_y
                    marker.pose.position.x = 0.
                    marker.pose.position.y = 0.
                    marker.pose.position.z = 0.1
                    marker.id = marker_id
                    marker_id += 1
                    marker.type = Marker.ARROW
                    marker.points.append(start_point)
                    marker.points.append(end_point)
                    marker.scale.x = 0.05
                    marker.scale.y = 0.1
                    marker.scale.z = 0.2
                        
                    self.marker_pub.publish(marker)                           

                    # <self.confidence_percentile>% confidence bounds of person's position as an ellipse:
                    cov = person.filtered_state_covariances[0][0] + person.var_obs # cov_xx == cov_yy == cov
                    std = cov**(1./2.)
                    gate_dist_euclid = scipy.stats.norm.ppf(1.0 - (1.0-self.confidence_percentile)/2., 0, std)
                    marker.pose.position.x = ps.point.x 
                    marker.pose.position.y = ps.point.y                    
                    marker.type = Marker.SPHERE
                    marker.scale.x = 2*gate_dist_euclid
                    marker.scale.y = 2*gate_dist_euclid
                    marker.scale.z = 0.01   
                    marker.color.r = person.colour[0]
                    marker.color.g = person.colour[1]
                    marker.color.b = person.colour[2]            
                    marker.color.a = 0.1
                    marker.pose.position.z = 0.0
                    marker.id = marker_id 
                    marker_id += 1                    
                    self.marker_pub.publish(marker)                

        # Clear previously published people markers
        for m_id in xrange(marker_id, self.prev_person_marker_id):
            marker = Marker()
            marker.header.stamp = now                
            marker.header.frame_id = self.publish_people_frame
            marker.ns = "People_tracked"
            marker.id = m_id
            marker.action = marker.DELETE   
            self.marker_pub.publish(marker)
        
        self.prev_person_marker_id = marker_id          

        # Publish people tracked message
        self.people_tracked_pub.publish(people_tracked_msg)
        self.polar_grid.publish_fov(now)         

if __name__ == '__main__':
    rospy.init_node('targetted_human_tracker', anonymous=True)
    t = Tracker()






from deception_movement_manager import DeceptionMovementManager
from behavior_with_deception.srv import DeceptiveCommandResponse, DeceptiveCommandRequest, DeceptiveCommand
from behavior_with_deception.msg import Deception

# from game_navigation.msg import PlayerInfo
from player_tracker.msg import TowerArray
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
from avoidance import FuzzyAvoider
from SafetyEvaluator import SafetyEvaluator
from KFfiltering import EKF

from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import rospy
import copy
import tf
import itertools

class Navigation:

    def __init__(self, kp, max_dot_theta, max_acc, t_max, ks, angle_deadzone, max_vel, min_vel, tower1, tower2, tower3, tower4, near_goal_distance,
                 proximity_threeshold, dontcare, rr_lower_bound, r_lower_bound, fr_lower_bound, fl_lower_bound, l_lower_bound, rl_lower_bound, unsafe_cmd_pub):
        self.KP = kp                                        # Proportional Gain (anglePControllerCallback())
        self.MAX_DOT_THETA = max_dot_theta                  # Maximum allowed angular velocity (in rad/sec)
        self.MAX_ACC = max_acc                              # Maximum allowed acceleration in the velocity smoother (avoid slippage)
        self.T_MAX = t_max                                  # Time constant for the velocity smoother
        self.KS = ks                                        # Gain factor of the velocity smoother
        self.ANGLE_DEADZONE = angle_deadzone                # DeadZone range (in rad/sec), values in this range are considered zero
        self.MAX_VEL = max_vel                              #(rosparam_get max_vel) Maximum desired velocity
        self.MIN_VEL = min_vel                              #(rosparam_get min_vel) Minimum desired velocity
        self.TOWER1 = tower1                                # Tower1 x,y coordinate
        self.TOWER2 = tower2                                # Tower2 x,y coordinate
        self.TOWER3 = tower3                                # Tower3 x,y coordinate
        self.TOWER4 = tower4                                # Tower4 x,y coordinate
        self.NEAR_GOAL_DISTANCE = near_goal_distance        # robot-tower distance that activates near_goal.
        self.PROXIMITY_THREESHOLD = proximity_threeshold    # If an obstacle is sensed below this threeshold it is said to be in "proximity condition"
        self.DONTCARE = dontcare                            # Obstacle in proximity condition but still don't affect the navigation trajectory
        self.RR_LOWER_BOUND  = rr_lower_bound               # Rear Right laser sector lower bound
        self.R_LOWER_BOUND = r_lower_bound                  # Right laser sector lower bound
        self.FR_LOWER_BOUND = fr_lower_bound                # Front Right laser sector lower bound
        self.FL_LOWER_BOUND = fl_lower_bound                # Front Left laser sector lower bound
        self.L_LOWER_BOUND = l_lower_bound                  # Left laser sector lower bound
        self.RL_LOWER_BOUND = rl_lower_bound                # Rear Left laser sector lower bound
        self.TOWERS = (self.TOWER1,self.TOWER2,             # list of towers
                       self.TOWER3, self.TOWER4) 
    
        self.current_vel = Twist()
        self.current_goal = 1
        self.current_angle_diff = 0
        # self.current_player_info = PlayerInfo()
        self.current_tower_positions = TowerArray()
        self.time_stamp = 0
        self.current_scan = LaserScan()
        self.current_scan_player = LaserScan()
        self.current_scan_obstacles = LaserScan()
        
        self.br = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        
        robot_init_pose = self.getRobotPose()
        self.robot_estimated_pose = np.array([[robot_init_pose[0]],[robot_init_pose[1]],[robot_init_pose[2]],[0.],[0.],[0.]])

        self.fuzzy_avoider = FuzzyAvoider(self.RR_LOWER_BOUND,self.R_LOWER_BOUND,
                                          self.FR_LOWER_BOUND,self.FL_LOWER_BOUND,
                                          self.L_LOWER_BOUND,self.RL_LOWER_BOUND)
        self.ekf = EKF(self.robot_estimated_pose)

        self.U_bar = np.array([[0.],[0.],[0.]])
        self.is_safe = True
        self.last_cmd_vel = Twist()         #last cmd_vel given

        self.safety_evaluator = SafetyEvaluator(self.DONTCARE)
        self.lock_rotation = False

        self.unsafe_cmd_pub = unsafe_cmd_pub

        # For DECEPTION
        self.deception_navigation_manager = DeceptionMovementManager()
        self.srv = rospy.Service('/behavior_with_deception/make_deception', DeceptiveCommand, self.make_deception_callback)
        self.pub_markers = rospy.Publisher('trajectory_marker_array', MarkerArray, queue_size=1)
        self.pub_single_marker = rospy.Publisher('single_marker_trajectory', Marker, queue_size=1)
        self.markerArray = MarkerArray()


    def set_max_speed(self,value):
        self.MAX_VEL = value
    
    def set_max_acc(self,value):
        self.MAX_ACC = value

    def set_ks(self,value):
        self.KS = value

    def velCallback(self,msg):
        """
        Updates current robot velocity (robot frame)
        """
        self.current_vel = copy.deepcopy(msg)

    def goalCallback(self,msg):
        """
        Updates current goal (target tower).
        """
        self.current_goal = msg.tower_number - 1  # we interpret this as an index for TOWERS list. Hence, the subtraction.

    def make_deception_callback(self, req):
        ''' 
        Service callback used when deception has been activated. 
        '''
        self.deception_navigation_manager.set_targets(req.real_target, req.fake_target)
        self.deception_navigation_manager.set_type_deception(req.type)   
        self.deception_navigation_manager.set_targets_coordinates(self.TOWERS)

        robot_coordinates = self.getRobotPose()

        self.deception_navigation_manager.set_robot_coordinates(np.array([robot_coordinates[0], robot_coordinates[1]]))
        self.deception_navigation_manager.set_playground_center(self.get_playground_center())
       
        points_trajectory = self.deception_navigation_manager.trajectory_planner()

        id_count = 0

        # For managing the idle when playing
        if self.deception_navigation_manager.type_deception == -2:
            starting_time = rospy.get_rostime()
            while( ( rospy.get_rostime() - starting_time ).to_sec() < 2):
                pass
                
        else:
            rospy.loginfo("Deception type:{} fake target:{} real_target:{}".format(self.deception_navigation_manager.type_deception, self.deception_navigation_manager.fake_target, self.deception_navigation_manager.real_target))

            for i in list(points_trajectory):
                
                starting_time = rospy.get_rostime()
                self.unsafe_cmd_pub.publish(self.navigate(i))
                deception_status = self.checking_trajectory_status(i, starting_time)
                self.markerArray.markers.append(self.create_marker(i, id_count))
                
                id_count+=1
                
                if not deception_status:
                    response = DeceptiveCommandRequest.FAILED
                    rospy.logwarn("Aborting deception..")
                    return DeceptiveCommandResponse(response)

            self.pub_markers.publish(self.markerArray)

        response = DeceptiveCommandRequest.SUCCEEDED
        return DeceptiveCommandResponse(response)

    def scan_obstacle_callback(self, msg):
        self.current_scan_obstacles = copy.deepcopy(msg) #msg.ranges

    def scanCallback(self,msg):
        """
        Gets distance measurements from the deployed lasers. These information are taken from /scan topic.
        OUTPUTS:
        @ current_range: a numpy array of length 1000 containing the measurements from each laser ray.
        """
        self.current_scan = copy.deepcopy(msg) #msg.ranges
        
    def scanPlayerCallback(self,msg):
        self.current_scan_player = copy.deepcopy(msg);

    def angleCallback(self,msg):
        """
        Updates current camera off-center player angle
        """
        self.current_angle_diff = msg.data
        
    # def playerInfoCallback(self,msg):
    #     """
    #     Updates current camera-player distance
    #     """
    #     self.current_player_info = copy.deepcopy(msg)

    def tpos_callback(self, msg):
        """Laser estimated tower position callback"""
        self.current_tower_positions = msg
        # self.update_tower_positions()

    def tower_rectangle_callback(self, msg):
        self.update_tower_positions(msg.polygon.points)

    def player_pos_callback(self, msg):
        self.player_angle = np.arctan2(msg.point.y, msg.point.x)

    # def anglePController(self):
    #     """
    #     The proportional controller callback to adjust robot 
    #     orientation in order to track the human player
    #     OUTPUT:
    #     @ U3: output of the proportional controller, opportunely clamped, that will be sent to ROS as a
    #             angular.cmd.vel that will allows the robot to rotate in order to track the player during
    #             the game. 
    #             PLEASE NOTICE: this comand does NOT require any change of coordinates, during the code we 
    #             may refer to it also as "bar_u3" or "bar_u3R" for consistency reasons but in fact we are 
    #             always considering the output value of this function. 
    #     """
    #     if(self.lock_rotation):
    #         return 0
    #     # Dead zone (Jerk-smother) used in order to eliminate angular
    #     # jerking while tracking
    #     if abs(self.current_angle_diff) < self.ANGLE_DEADZONE:
    #         self.current_angle_diff = 0
            
    #     # Proportional Controller
    #     dot_theta = self.KP*self.current_angle_diff
    #     if (self.current_player_info.distance < 1) and (abs(self.current_player_info.header.stamp.to_sec() - rospy.Time.now().to_sec()) < 1.5): 
    #         # the condition is activated when the player is within 1 meter from the camera and when the received
    #         # message is no older than 1.5 sec. The more the player is close the more the angular rotation command is smoothed
    #         dot_theta = dot_theta * self.current_player_info.distance

    #     # Angular velocity clamping (max angular velocity in rad/sec)
    #     if dot_theta >= self.MAX_DOT_THETA:
    #         return self.MAX_DOT_THETA
    #     elif dot_theta <= -self.MAX_DOT_THETA:
    #         return -self.MAX_DOT_THETA
    #     else:
    #         return dot_theta

    def velocity_smoother(self, robot_unsmoothed_cmd_vel, robot_vel):
        """
        Receives as input [bar_u1R, bar_u2R] that is the vector of the unsmoothed
        cmd_vel (x and y velocity set points) and perform a smoothing on the cmd_vel
        signals that are sent to ROS (U1, U2) in order to avoid step transitions.
        Step transition on the velocity set point can cause wheels' slippage and 
        loss of localization. 
        INPUTS:
            @ bar_u1R: x-unsmoothed cmd_vel (robot frame)
            @ bar_u2R: y-unsmoothed cmd_vel (robot frame)
            @ robot_vel: vector of the robot actual linear xy-velocities (robot frame)
        OUTPUTS:
            @ smoothed_cmd_vel: vector of smoothed velocity comands that are sent to ROS (robot frame)
        """


        initial_vel = [0,0]
        if self.is_safe:
            initial_vel = [robot_vel[0], robot_vel[1]]

        # define acceleration
        initial_acc = [robot_unsmoothed_cmd_vel[0] - initial_vel[0], robot_unsmoothed_cmd_vel[1] - initial_vel[1]]

        # X-accelerations clamping
        if initial_acc[0] >= self.MAX_ACC:
            initial_acc[0] = self.MAX_ACC
        elif initial_acc[0] <= -self.MAX_ACC:
            initial_acc[0] = -self.MAX_ACC

        # Y-accelerations clamping
        if initial_acc[1] >= self.MAX_ACC:
            initial_acc[1] = self.MAX_ACC
        elif initial_acc[1] <= -self.MAX_ACC:
            initial_acc[1] = -self.MAX_ACC

        # generate interpolating polynomial and cmd_vel
        t1 = abs(1 / self.KS * (self.T_MAX) / (self.MAX_VEL))
        t2 = abs(1 / self.KS * (self.T_MAX) / (self.MAX_VEL))

        # smoothed cmd_vel
        return (initial_vel[0] + initial_acc[0] * t1, initial_vel[1] + initial_acc[1] * t2)


    def kinematic_compensator(self):
        """
        Transform the unsmoothed cmd_vel [bar_u1, bar_u2] from world frame
        to robot reference frame. This allows to keep linear and angular movements
        of the robot decoupled.
        INPUTS:
            @ bar_u1:   unsmoothed x-linear.cmd.vel (world frame)
            @ bar_u2:   unsmoothed y-linear.cmd.vel (world frame)
            @ theta_hat: estimated robot orientation angle
        OUTPUTS:
            @ U_barR: vector containing unsmoothed xy-linear.cmd.vels (robot frame)
        """
        # G matrix converts from robot to world frame
        G = np.array([[np.cos(self.robot_estimated_pose[2][0]), -np.sin(self.robot_estimated_pose[2][0]), 0.],
                    [np.sin(self.robot_estimated_pose[2][0]),  np.cos(self.robot_estimated_pose[2][0]), 0.],
                    [0., 0., 1.]])

        # iG matrix converts from world to robot frame
        iG = np.linalg.inv(G)
        
        # convert velocity commands from world to robot frame
        U_barR = np.dot(iG, self.U_bar)
        
        # output the velocity command in robot frame
        return U_barR       
    

    def towerNavigation(self, goal = None):
        """ 
        Implements navigation to Target towers through the playground
        INPUTS:
            @ robot_estimated_pose: a 3D-vector representing the robot estimated pose ([x_hat, y_hat, theta_hat])
            @         current_goal: xy-position of the targeted tower
            @              is_safe: boolean variable that become FALSE if the robot is approaching an obstacle
        OUTPUTS:
            @     U_bar: vector of unsmoothed xy-linear.cmd.vel (world frame)
            @ is_near_goal: boolean variable that become TRUE if the robot is close to a targeted tower
        """

        if goal is None :
            target_x = self.TOWERS[self.current_goal][0]
            target_y = self.TOWERS[self.current_goal][1]
        else:
            target_x = goal[0]
            target_y = goal[1]

        # if abs(self.current_tower_positions.header.stamp.to_sec() - rospy.get_rostime().to_sec()) > 1:
        #     # define initial e final point when the robot receive the id of the targeted tower
        #     xd = (self.robot_estimated_pose[0][0], self.TOWERS[self.current_goal][0])
        #     yd = (self.robot_estimated_pose[1][0], self.TOWERS[self.current_goal][1])
        # else:
        # define initial e final point when the robot receive the id of the targeted tower
        xd = (self.robot_estimated_pose[0][0], target_x)
        yd = (self.robot_estimated_pose[1][0], target_y)

        # DEBUG set tower 1 as goal
        # xd = (self.robot_estimated_pose[0][0], self.TOWERS[0][0])
        # yd = (self.robot_estimated_pose[1][0], self.TOWERS[0][1])

        # define the robot deviation from the required trajectory
        delta_x = xd[1] - xd[0]
        delta_y = yd[1] - yd[0]

        # generates the direction of the motion based on the euclidian distance from goal
        alpha = np.arctan2(delta_y, delta_x)

        # check if the robot is near its goal (this will change in obstacle avoidance behaviour)
        goal_distance = (delta_x**2 + delta_y**2)**0.5
        
        # set is_near_goal
        is_near_goal = False
        if goal_distance < self.NEAR_GOAL_DISTANCE:
            is_near_goal = True

        # SAFETY CHECK: the controller will generates cmd_vel commands only if the safety condition is satisfied
        # if safety condition is satisfied then: enable == 1;
        if self.is_safe == True:
            self.U_bar[0] = self.MAX_VEL*np.cos(alpha)
            self.U_bar[1] = self.MAX_VEL*np.sin(alpha)

        return is_near_goal
    
    def pubRobotFilteredPose(self, pose):
        """ Publishes its Kalman filtered position"""
        self.br.sendTransform((pose[0][0], pose[1][0], 0),
                        tf.transformations.quaternion_from_euler(0, 0, pose[2][0]),
                        rospy.Time.now(),
                        '/base_link_kalman_filtered',
                        "/map")

    def getRobotPose(self):
        """
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform('/map','/base_link', rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))
            # transform from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
 
            return np.array([trans[0], trans[1], euler[2]])   # [xR,yR,theta]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))

    def get_playground_center(self):
        """
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform('/map','/playground_center', rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform('/map','/playground_center', rospy.Time(0))
 
            return np.array([trans[0], trans[1]])   # [xR,yR]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))


    def evaluateColision(self, goal = None):
        """Set is_safe variable"""

        alpha = np.arctan2(self.last_cmd_vel.linear.y, self.last_cmd_vel.linear.x)

        if self.player_angle != None:
            delta_angle = abs(self.player_angle - alpha)

            if delta_angle > (np.pi / 2):
                delta_angle = (2*np.pi) - delta_angle

            if delta_angle < (20 * (np.pi / 180)):
                # check if an obstacle is detected below the proximity threeshold
                proximity = False
                for values in self.current_scan_obstacles.ranges:
                    if values < self.PROXIMITY_THREESHOLD:
                        proximity = True
                        break

                # NOTE:  Why do we have proximity and dontcare as well thersholds? I think Davide meant to use 'proximity'
                # as a way to start reducing the efect of inertia in case we have to suddenly stop because of a 'dontcare'
                # condition.  Thus, 'proximity' is use just to start reducing the robot action and preparing it to a possible
                # obstacle avoidance.

                # perform a safety check (NOTE: Does not care for direction of movement)
                # dontcare_condition = np.array(self.current_scan.ranges) < self.DONTCARE

                #dontcare_condition = self.dontcare_condition_evaluation()
                #dontcare_condition = self.dontcare()

                filtered_scan = self.filter_scan(self.current_scan_obstacles.ranges)
            
                self.set_evaluator_mode(goal = goal)
                self.is_safe = self.safety_evaluator.evaluate_safety(filtered_scan, self.current_scan_obstacles)
                rospy.loginfo("Safety condition: {}".format(self.is_safe))

    
    def laserScanManager(self):
        """
        Process the informations coming from the laser scan (topic: /scan), divide the scanned area around the robot into
        'sectors' (rear right, right, front right, front left, left, rear left) and for each sector compute the minimum
        detected distance from obstacles and the index of each computed minimum. 
        This function also manage the value of 'is_safe' value deciding whether to perform the navigation using 'tower_navigation'
        (is_safe = TRUE) or 'obstacleAvoidance' (is_safe = FALSE).
        
        INPUTS:
            @ current_range: a numpy array of length 1000 containing the measurements from each laser ray.
        OUTPUTS:
            @ is_safe: boolean variable that become FALSE if the robot is approaching an obstacle
        OUTPUTS (sent to the fuzzy avoider) Tuples containing minimum and index of the minimum for each sector    
            @ rear_right
            @ right
            @ front_right
            @ front_left
            @ left
            @ rear_left 
        """
        
        '''
        DAVIDE OBSTACLE AVOIDANCE
         # check if an obstacle is detected below the proximity threeshold in the player scan
        if(self.is_safe):
            # If we come from a safe condition check only player
            scan_to_check_for_safety = self.current_scan_player
        else:
            # Check all scan
        	scan_to_check_for_safety = self.current_scan
        	
        	
        proximity = False
        for values in scan_to_check_for_safety.ranges:
            if values < self.PROXIMITY_THREESHOLD:
                proximity = True
                break

        # perform a safety check
        dontcare_condition = np.array(scan_to_check_for_safety.ranges) < self.DONTCARE
        new_is_safe = True
        if proximity and (all(dontcare_condition) == False) and (len(dontcare_condition) != 0):
            new_is_safe = False
	    self.is_safe = new_is_safe
        '''
        # Generate sensing areas: rear right, right, front right, front left, left, rear left in the WHOLE SCAN
        rear_right_sec  = self.current_scan.ranges[0:149]
        right_sec       = self.current_scan.ranges[150:309]
        front_right_sec = self.current_scan.ranges[310:499]
        front_left_sec  = self.current_scan.ranges[500:689]
        left_sec        = self.current_scan.ranges[690:859]
        rear_left_sec   = self.current_scan.ranges[850:999]

        # Get the minimum value for each sensed area
        min_rear_right  = min(rear_right_sec)
        min_right       = min(right_sec)
        min_front_right = min(front_right_sec)
        min_front_left  = min(front_left_sec)
        min_left        = min(left_sec)
        min_rear_left   = min(rear_left_sec)

       
        # Get the index of the minimum for each area
        min_rear_right_index  = rear_right_sec.index(min_rear_right)
        min_right_index       = right_sec.index(min_right)
        min_front_right_index = front_right_sec.index(min_front_right)
        min_front_left_index  = front_left_sec.index(min_front_left)
        min_left_index        = left_sec.index(min_left)
        min_rear_left_index   = rear_left_sec.index(min_rear_left)

        

        # Generates outputs for fuzzyfication:
        rear_right  = (min_rear_right, min_rear_right_index)
        right       = (min_right, min_right_index)
        front_right = (min_front_right, min_rear_right_index)
        front_left  = (min_front_left, min_front_left_index)
        left        = (min_left, min_left_index)
        rear_left   = (min_rear_left, min_rear_left_index)

        return rear_right, right, front_right, front_left, left, rear_left

    
    def filter_scan(self, scan):
        filtered_values = []
        self.danger = False
        self.search = False
        for value in self.current_scan_obstacles.ranges:
            if value < 0.8:
                self.search = True
                if value < self.DONTCARE:
                    self.danger = True
                filtered_values.append(value)
            else:
                filtered_values.append(0)
        
        return filtered_values

    def set_evaluator_mode(self, goal = None):
        if(self.is_near_goal(goal = goal) and goal is None):
            self.lock_rotation = True
            if(self.danger):
                self.safety_evaluator.set_mode(SafetyEvaluator.APPROACHING_TOWER)
                # rospy.debug("APPROACHING TOWER")
            elif(self.search): # Needed to avoid to pass an empty filtered scan 
                # rospy.debug("SEARCHING TOWER")
                self.safety_evaluator.set_mode(SafetyEvaluator.SEARCHING_TOWER)
                # TODO IMPROVEMENT
                self.safety_evaluator.set_searching_angle(self.estimate_tower_angle(goal = goal))
            else:
                self.safety_evaluator.set_mode(SafetyEvaluator.DEFAULT)
        else:
            self.lock_rotation = False
            self.safety_evaluator.set_mode(SafetyEvaluator.DEFAULT)
            
                
    def is_near_goal(self, goal = None):
        if goal is None :
            target_x = self.TOWERS[self.current_goal][0]
            target_y = self.TOWERS[self.current_goal][1]
            distance_max = 0.8
        else:
            target_x = goal[0]
            target_y = goal[1]
            distance_max = 0.1


        # define initial e final point when the robot receive the id of the targeted tower
        xd = (self.robot_estimated_pose[0][0], target_x)
        yd = (self.robot_estimated_pose[1][0], target_y)

        # DEBUG GO TO TOWER 1 #
        # xd = (self.robot_estimated_pose[0][0], self.TOWERS[0][0])
        # yd = (self.robot_estimated_pose[1][0], self.TOWERS[0][1])

        # define the robot deviation from the required trajectory
        delta_x = xd[1] - xd[0]
        delta_y = yd[1] - yd[0]

        goal_distance = (delta_x**2 + delta_y**2)**0.5

        # rospy.logwarn('Goal_distance: {}'.format(goal_distance))

        if goal_distance < distance_max:
            return True
        return False
        
    def check_distances_are_similar(self, d1, d2, acceptance_thd=0.1):       
        delta = abs(d1 - d2)
        if( delta < acceptance_thd ):
            return True
        return False

    def estimate_tower_angle(self, goal = None):
        ''' Estimates the angle of the target tower wrt the robot '''
        pose = self.getRobotPose()
        rot_to_map = pose[2]
        rospy.logwarn("Rotation to Map: {}".format(rot_to_map))

        if goal is None:
            target_x = self.TOWERS[self.current_goal][0]
            target_y = self.TOWERS[self.current_goal][1]
        else:
            target_x = goal[0]
            target_y = goal[1]

        rot_to_target = np.arctan2(pose[1] - target_y, pose[0] - target_x)

        # DEBUG 
        # rot_to_target = np.arctan2(pose[1] - self.TOWERS[0][1], pose[0] - self.TOWERS[0][0])
        rospy.logwarn("Rotation to Target: {}".format(rot_to_target))

        adjusting_factor = np.pi # Angle 0 for the laser scan is behind the robot

        angle = rot_to_map + rot_to_target + adjusting_factor
        rospy.logwarn("Total angle rad: {}".format(angle))
        rospy.logwarn("Total angle deg: {}".format(np.rad2deg(angle)))
        
        if angle > 2*np.pi:
            return angle - 2*np.pi

        return angle

    def navigate(self, goal = None):
        """
        goal = None --> robot navigates to the tower with Davide's navigation
        """

        # get current robot_velocity (/base_link /vel) in world_frame (/map)
        robot_vel = np.array([[self.current_vel.linear.x],           # linear x-velocity (robot frame)
                             [self.current_vel.linear.y],           # linear y-velocity (robot frame)
                             [self.current_vel.angular.z]])        # angular velocity  (robot frame)

        # G matrix converts from robot to world frame
        G = np.array([[np.cos(self.robot_estimated_pose[2][0]), -np.sin(self.robot_estimated_pose[2][0]), 0],
                    [np.sin(self.robot_estimated_pose[2][0]),  np.cos(self.robot_estimated_pose[2][0]), 0],
                    [0, 0, 1.]])

        # convert velocity commands from robot to world frame
        robot_world_vel = np.dot(G, robot_vel)
        
        robot_pose = self.getRobotPose()
        
        # EKF
        self.robot_estimated_pose = self.ekf.predict(robot_pose,self.U_bar)
        self.pubRobotFilteredPose(self.robot_estimated_pose)
        self.ekf.update(robot_pose, robot_world_vel)

        # use the global planner (TOWER NAVIGATION)
        if goal is None:
            is_near_goal = self.towerNavigation()
        else:
            is_near_goal = self.towerNavigation(goal = goal)
        
        # use kinematic compensator
        robot_unsmoothed_cmd_vel = self.kinematic_compensator()
        
        # velocity smoothing
        smoothed_cmd_vel = self.velocity_smoother(robot_unsmoothed_cmd_vel, robot_vel)

        # updates is_safe and process /scan information
        rear_right, right, front_right, front_left, left, rear_left = (None,None,None,None,None,None)
        
        if len(self.current_scan_obstacles.ranges) != 0:
           if goal is None:
               self.evaluateColision()
           else:
               self.evaluateColision(goal = goal)

        # Fuzzy
        if len(self.current_scan_player.ranges) != 0 and len(self.current_scan.ranges) != 0:
            rear_right, right, front_right, front_left, left, rear_left = self.laserScanManager()
            
        if not self.is_safe:
            rear_right, right, front_right, front_left, left, rear_left = self.laserScanManager()
            smoothed_cmd_vel = self.fuzzy_avoider.avoidObstacle(rear_right, right, front_right, front_left, left, rear_left, is_near_goal)

        unsafe_msg = Twist()
        unsafe_msg.linear.x = smoothed_cmd_vel[0]
        unsafe_msg.linear.y = smoothed_cmd_vel[1]
        unsafe_msg.angular.z = 0#self.anglePController()

        # save last cmd given
        self.last_cmd_vel = unsafe_msg;
        
        return unsafe_msg


    def convert_to_map(self, point, header):
        rospy.loginfo("Converting..")

        ps = PointStamped()
        ps.header.frame_id = 'base_link'
        ps.header.stamp = header.stamp
        ps.point.x = point.x
        ps.point.y = point.y

        converted = self.tf_listener.transformPoint('map', ps)
        return converted.point

    def update_tower_positions(self, vertices):
        for vertex in vertices:
            index = self.match_tower_index(vertex)
            self.TOWERS[index][0] = vertex.x
            self.TOWERS[index][1] = vertex.y
        
        # for i,tower in enumerate(self.TOWERS):
        #     rospy.loginfo("Tower {} is at x:{}, y:{}".format(i+1, tower[0], tower[1]))

    def match_tower_index(self, vertex):
        # Match the closest tower with the point given
        distances = [ ((vertex.x - tower_pos[0])**2 + (vertex.y - tower_pos[1])**2) for tower_pos in (self.TOWERS)]
        matched_tower_index = np.argmin(distances)
        return matched_tower_index

    def checking_trajectory_status(self, point, starting_time):
        ''' 
        Function used for checking if the robo has arrived to the published point.
        Used ONLY with DECEPTION.
        It also checks if the robot arrives in a reasonable time, calculated using get_eta.
        If the robot arrives on time -- > return TRUE so the DECEPTION procdure can continue
        if the time is over -- > return FALSE so the DECEPTION procedure has failed and it has to stop
        '''
        id_count = 0
        estimated_time = self.get_eta(point)
        while (not self.is_near_goal(point)):
            current_time = rospy.get_rostime()
            if(current_time - starting_time).to_sec() > estimated_time:
                return False
            self.pub_single_marker.publish(self.create_marker(point, id_count))
            self.unsafe_cmd_pub.publish(self.navigate(point))
            id_count += 1
        return True

    def get_eta(self, point):
        '''
        The function calculates and retuns the estimated time arrival for reaching a trajectory's point.
        Knowing the ETA lets us to detect when a deception has failed
        '''
        robot_position = self.getRobotPose()
        vel_estimated = (self.MAX_VEL + self.MIN_VEL) / 2
        delta_x = robot_position[0] - point[0]
        delta_y = robot_position[1] - point[1]
        distance = (delta_x**2 + delta_y**2)**0.5
        estimated_time = distance / vel_estimated
        return estimated_time + 5

    def create_marker(self, point, id_count):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.header.stamp = rospy.get_rostime()
        marker.id = id_count
        marker.type = marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1] 
        marker.pose.position.z = 0.2
        return marker      

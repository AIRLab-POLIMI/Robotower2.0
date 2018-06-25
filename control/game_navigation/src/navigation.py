from behavior_manager.msg import Goal
from kinect_tracker.msg import PlayerInfo
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from avoidance import FuzzyAvoider
from KFfiltering import EKF
import numpy as np
import rospy
import copy
import tf

class Navigation:

    def __init__(self, kp, max_dot_theta, max_acc, t_max, ks, angle_deadzone, max_vel, tower1, tower2, tower3, tower4, near_goal_distance,
                 proximity_threeshold, dontcare, rr_lower_bound, r_lower_bound, fr_lower_bound, fl_lower_bound, l_lower_bound, rl_lower_bound):
        self.KP = kp                                    # Proportional Gain (anglePControllerCallback())
        self.MAX_DOT_THETA = max_dot_theta              # Maximum allowed angular velocity (in rad/sec)
        self.MAX_ACC = max_acc                          # Maximum allowed acceleration in the velocity smoother (avoid slippage)
        self.T_MAX = t_max                              # Time constant for the velocity smoother
        self.KS = ks                                    # Gain factor of the velocity smoother
        self.ANGLE_DEADZONE = angle_deadzone            # DeadZone range (in rad/sec), values in this range are considered zero
        self.MAX_VEL = max_vel                          #(rosparam_get max_vel) Maximum desired velocity
        self.TOWER1 = tower1                            # Tower1 x,y coordinate
        self.TOWER2 = tower2                            # Tower2 x,y coordinate
        self.TOWER3 = tower3                            # Tower3 x,y coordinate
        self.TOWER4 = tower4                            # Tower4 x,y coordinate
        self.NEAR_GOAL_DISTANCE = near_goal_distance    # robot-tower distance that activates near_goal.
        self.PROXIMITY_THREESHOLD = proximity_threeshold# If an obstacle is sensed below this threeshold it is said to be in "proximity condition"
        self.DONTCARE = dontcare                        # Obstacle in proximity condition but still don't affect the navigation trajectory
        self.RR_LOWER_BOUND  = rr_lower_bound           # Rear Right laser sector lower bound
        self.R_LOWER_BOUND = r_lower_bound              # Right laser sector lower bound
        self.FR_LOWER_BOUND = fr_lower_bound            # Front Right laser sector lower bound
        self.FL_LOWER_BOUND = fl_lower_bound            # Front Left laser sector lower bound
        self.L_LOWER_BOUND = l_lower_bound              # Left laser sector lower bound
        self.RL_LOWER_BOUND = rl_lower_bound            # Rear Left laser sector lower bound
        self.TOWERS = (self.TOWER1,self.TOWER2,         # list of towers
                       self.TOWER3, self.TOWER4) 
    
        self.current_vel = Twist()
        self.current_goal = 1
        self.current_angle_diff = 0
        self.current_player_info = PlayerInfo()
        self.time_stamp = 0
        self.current_range = LaserScan()
        
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

    def scanCallback(self,msg):
        """
        Gets distance measurements from the deployed lasers. These information are taken from /scan topic.
        OUTPUTS:
        @ current_range: a numpy array of length 1000 containing the measurements from each laser ray.
        """
        self.current_range = copy.deepcopy(msg) #msg.ranges

    def angleCallback(self,msg):
        """
        Updates current camera off-center player angle
        """
        self.current_angle_diff = msg.data
        
    def playerInfoCallback(self,msg):
        """
        Updates current camera-player distance
        """
        self.current_player_info = copy.deepcopy(msg)


    def anglePController(self):
        """
        The proportional controller callback to adjust robot 
        orientation in order to track the human player
        OUTPUT:
        @ U3: output of the proportional controller, opportunely clamped, that will be sent to ROS as a
                angular.cmd.vel that will allows the robot to rotate in order to track the player during
                the game. 
                PLEASE NOTICE: this comand does NOT require any change of coordinates, during the code we 
                may refer to it also as "bar_u3" or "bar_u3R" for consistency reasons but in fact we are 
                always considering the output value of this function. 
        """
        # Dead zone (Jerk-smother) used in order to eliminate angular
        # jerking while tracking
        if abs(self.current_angle_diff) < self.ANGLE_DEADZONE:
            self.current_angle_diff = 0
            
        # Proportional Controller
        dot_theta = self.KP*self.current_angle_diff
        if (self.current_player_info.distance < 1) and (abs(self.current_player_info.header.stamp.to_sec() - rospy.Time.now().to_sec()) < 1.5): 
            # the condition is activated when the player is within 1 meter from the camera and when the received
            # message is no older than 1.5 sec. The more the player is close the more the angular rotation command is smoothed
            dot_theta = dot_theta * self.current_player_info.distance

        # Angular velocity clamping (max angular velocity in rad/sec)
        if dot_theta >= self.MAX_DOT_THETA:
            return self.MAX_DOT_THETA
        elif dot_theta <= -self.MAX_DOT_THETA:
            return -self.MAX_DOT_THETA
        else:
            return dot_theta

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

    def towerNavigation(self):
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

        # define initial e final point when the robot receive the id of the targeted tower
        xd = (self.robot_estimated_pose[0][0], self.TOWERS[self.current_goal][0])
        yd = (self.robot_estimated_pose[1][0], self.TOWERS[self.current_goal][1])

        # define the robot deviation from the required trajectory
        delta_x = xd[1] - xd[0]
        delta_y = yd[1] - yd[0]

        # generates the direction of the motion based on the euclidian distance from goal
        alpha = np.arctan2(delta_y, delta_x)

        # check if the robot is near its goal (this will change in obstacle avoidance behaviour)
        goal_distance = (delta_x**2 + delta_y**2)**0.5
        
        # set is_near_goal
        is_near_goal = 0
        if goal_distance < self.NEAR_GOAL_DISTANCE:
            is_near_goal = 1

        # SAFETY CHECK: the controller will generates cmd_vel commands only if the safety condition is satisfied
        # if safety condition is satisfied then: enable == 1;
        if self.is_safe == 1:
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

    def laserScanManager(self):
        """
        Process the informations coming from the laser scan (topic: /scan), divide the scanned area around the robot into
        'sectors' (rear right, right, front right, front left, left, rear left) and for each sector compute the minimum
        detected distance from obstacles and the index of each computed minimum. 
        This function also manage the value of 'is_safe' value deciding wheter to perform the navigation using 'tower_navigation'
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

        # check if an obstacle is detected below the proximity threeshold
        proximity = False
        for values in self.current_range.ranges:
            if values < self.PROXIMITY_THREESHOLD:
                proximity = True
                break

        # perform a safety check
        dontcare_condition = np.array(self.current_range.ranges) < self.DONTCARE
        new_is_safe = True
        if proximity and (all(dontcare_condition) == False) and (len(dontcare_condition) != 0):
            new_is_safe = False
        self.is_safe = new_is_safe
        
        # Generate sensing areas: rear right, right, front right, front left, left, rear left
        rear_right_sec  = self.current_range.ranges[0:149]
        right_sec       = self.current_range.ranges[150:309]
        front_right_sec = self.current_range.ranges[310:499]
        front_left_sec  = self.current_range.ranges[500:689]
        left_sec        = self.current_range.ranges[690:859]
        rear_left_sec   = self.current_range.ranges[850:999]

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
            

    def navigate(self):
        """
        TODO performs navigation
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
        is_near_goal = self.towerNavigation()
        
        # use kinematic compensator
        robot_unsmoothed_cmd_vel = self.kinematic_compensator()
        
        # velocity smoothing
        smoothed_cmd_vel = self.velocity_smoother(robot_unsmoothed_cmd_vel, robot_vel)

        # updates is_safe and process /scan information
        rear_right, right, front_right, front_left, left, rear_left = (None,None,None,None,None,None)
        
        if len(self.current_range.ranges) != 0:
            rear_right, right, front_right, front_left, left, rear_left = self.laserScanManager()

        # Fuzzy
        if not self.is_safe:
            smoothed_cmd_vel = self.fuzzy_avoider.avoidObstacle(rear_right, right, front_right, front_left, left, rear_left, is_near_goal)

        unsafe_msg = Twist()
        unsafe_msg.linear.x = smoothed_cmd_vel[0]
        unsafe_msg.linear.y = smoothed_cmd_vel[1]
        unsafe_msg.angular.z = self.anglePController()

        return unsafe_msg
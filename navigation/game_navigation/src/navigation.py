#!/usr/bin/env python
from behavior_manager.msg import Goal
from kinect_tracker.msg import PlayerInfo
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from obstacleAvoidance import fuzzyAvoider
import numpy as np
import rospy
import copy
import tf

KP = None               # Proportional Gain (anglePControllerCallback())
MAX_DOT_THETA = None    # Maximum allowed angular velocity (in rad/sec)
MAX_ACC = None          # Maximum allowed acceleration in the velocity smoother (avoid slippage)
T_MAX = None            # Time constant for the velocity smoother
KS = None               # Gain factor of the velocity smoother
ANGLE_DEADZONE = None   # DeadZone range (in rad/sec), values in this range are considered zero
MAX_VEL = None          #(rosparam_get max_vel) Maximum desired velocity
TOWER1 = None           # Tower1 x,y coordinate
TOWER2 = None           # Tower2 x,y coordinate
TOWER3 = None           # Tower3 x,y coordinate
TOWER4 = None           # Tower4 x,y coordinate
NEAR_GOAL_DISTANCE = None # robot-tower distance that activates near_goal.
PROXIMITY_THREESHOLD = None # If an obstacle is sensed below this threeshold it is said to be in "proximity condition"
DONTCARE = None         # Obstacle in proximity condition but still don't affect the navigation trajectory
TOWERS = []             # list of towers 

current_vel = Twist()
current_goal = 0
current_angle_diff = 0
current_range = ()

U_bar = np.array([[0.],[0.],[0.]])
is_safe = True

# KALMAN CONSTANTS
n = 6                                   # number of states
q = 0.6                                 # process std
r1 = 0.5                                # position measurement std
r2 = np.pi/36                           # orientation measurement std
T = 0.02                                # sampling time
Q = (q**2)*np.eye(n)                    # noise covariance matrix of the process (6x6 matrix)
R = np.array([[r1**2, 0, 0],            # covariance of measurements
              [0, r1**2, 0],
              [0, 0, r2**2]])

H = np.array([[1., 0, 0, 0, 0, 0],      # Observation Matrix and measurements vector
              [0, 1., 0, 0, 0, 0],
              [0, 0, 1., 0, 0, 0]])

B = np.array([[0, 0, 0],                # Control Matrix and Control inputs
              [0, 0, 0],
              [0, 0, 0],
              [1., 0, 0],
              [0, 1., 0],
              [0, 0, 1.]])


              
# KALMAN VARIABLES
robot_estimated_pose = np.array([[0.],           # initial x-position
                                 [0.],           # initial y-position
                                 [0.],           # initial orientation
                                 [0.],           # initial x-velocity
                                 [0.],           # initial y-velocity
                                 [0.]])          # initial angular-velocity

P = np.eye(n)                                    # initial covariance (6x6 matrix)


# Publishers
pub = rospy.Publisher('unsafe/cmd_vel', Twist, queue_size=1)

# Define tf listener
tf_listener = tf.TransformListener()

def velCallback(msg):
    """
    Updates current robot velocity (body frame)
    """
    global current_vel
    current_vel = copy.deepcopy(msg)

def goalCallback(msg):
    """
    Updates current goal (target tower).
    """
    global current_goal
    current_goal = msg.tower_number - 1  # we interpret this as an index for TOWERS list. Hence, the subtraction.


def angleCallback(msg):
    """
    Updates current camera off-center player angle
    """
    global current_angle_diff
    current_angle_diff = msg.angle

def anglePController():
    """
    The proportional controller callback to adjust robot 
    orientation in order to track the human player

    Output:
        TODO
    """

    global current_angle_diff

    # Dead zone (Jerk-smother) used in order to eliminate angular
    # jerking while tracking
    
    if abs(current_angle_diff) < ANGLE_DEADZONE:
        current_angle_diff = 0

    # Proportional Controller
    dot_theta = KP*current_angle_diff

    # Angular velocity clamping (max angular velocity in rad/sec)
    if dot_theta >= MAX_DOT_THETA:
        return MAX_DOT_THETA
    elif dot_theta <= -MAX_DOT_THETA:
        return -MAX_DOT_THETA
    else:
        return dot_theta

def velocity_smoother(robot_unsmoothed_cmd_vel, robot_vel):
    """
    Receives as input [bar_u1R, bar_u2R] that is the vector of the unsmoothed
    cmd_vel (x and y velocity set points) and perform a smoothing on the cmd_vel
    signals that are sent to ROS (U1, U2) in order to avoid step transitions.
    Step transition on the velocity set point can cause wheels' slippage and 
    loss of localization. 
    INPUTS:
    TODO
    """


    initial_vel = [0,0]
    if is_safe:
        initial_vel = [robot_vel[0], robot_vel[1]]

    # define acceleration
    initial_acc = [robot_unsmoothed_cmd_vel[0] - initial_vel[0], robot_unsmoothed_cmd_vel[1] - initial_vel[1]]

    # X-accelerations clamping
    if initial_acc[0] >= MAX_ACC:
        initial_acc[0] = MAX_ACC
    elif initial_acc[0] <= -MAX_ACC:
        initial_acc[0] = -MAX_ACC

    # Y-accelerations clamping
    if initial_acc[1] >= MAX_ACC:
        initial_acc[1] = MAX_ACC
    elif initial_acc[1] <= -MAX_ACC:
        initial_acc[1] = -MAX_ACC

    # generate interpolating polynomial and cmd_vel
    t1 = abs(1 / KS * (T_MAX) / (MAX_VEL))
    t2 = abs(1 / KS * (T_MAX) / (MAX_VEL))

    # smoothed cmd_vel
    return (initial_vel[0] + initial_acc[0] * t1, initial_vel[1] + initial_acc[1] * t2)


def kinematic_compensator():
    """
    Transform the unsmoothed cmd_vel [bar_u1, bar_u2, bar_u3] from world frame
    to robot reference frame. This allows to keep linear and angular movements
    of the robot decoupled.
    TODO
    """
    # G matrix converts from robot to world frame
    G = np.array([[np.cos(robot_estimated_pose[2][0]), -np.sin(robot_estimated_pose[2][0]), 0.],
                  [np.sin(robot_estimated_pose[2][0]),  np.cos(robot_estimated_pose[2][0]), 0.],
                  [0., 0., 1.]])

    # iG matrix converts from world to robot frame
    iG = np.linalg.inv(G)
    
    # convert velocity commands from world to robot frame
    U_barR = np.dot(iG, U_bar)
    
    # output the velocity command in robot frame
    return U_barR


# TODO change name of variable to more interpretable ones.
def holonomicEKFPrediction(robot_pose, U_bar):
    """
    Perform a filtering on the measurements returned from AMCL 
    considering model and measurements uncertainty.
    INPUTS:
        @ robot_pose: a 3D-vector representing the robot pose ([x, y, theta])
        @ U_bar: unsmoothed velocity control commands (from tower navigator and kinect_tracker) -- [ bar_u1, bar_u2, bar_u3] 
    OUTPUTS:
        @ robot_estimated_pose: a 3D-vector representing the robot estimated pose ([x_hat, y_hat, theta_hat])
    """

    global robot_estimated_pose, P, H, B

    # Transition Matrix (theta is in rad)
    chi_x = (-np.sin(robot_pose[2]) / (2 * np.sin(np.pi / 3)) + (np.cos(robot_pose[2]) / (2 + 2 * np.cos(np.pi / 3))))
    chi_y = (-np.cos(robot_pose[2]) / (2 * np.sin(np.pi / 3)) + (np.sin(robot_pose[2]) / (2 + 2 * np.cos(np.pi / 3))))
    F = np.array([[1., 0, 0, chi_x*T, 0, 0],
                  [0, 1., 0, 0, chi_y*T, 0],
                  [0, 0, 1., 0, 0, T],
                  [0, 0, 0, 1., 0, 0],
                  [0, 0, 0, 0, 1., 0],
                  [0, 0, 0, 0, 0, 1.]])

    # A priori prediction
    robot_estimated_pose = np.dot(F, robot_estimated_pose) + np.dot(B, U_bar)
    P = np.dot(np.dot(F, P), F.transpose())

def holonomicEKFUpdate(robot_pose):
    """
    Kalman update algorithm
    """

    global P, robot_estimated_pose, H

    amcl_meas = np.array([[robot_pose[0]],
                          [robot_pose[1]],
                          [robot_pose[2]],
                          [0],
                          [0],
                          [0]])
   
    Z = np.dot(H, amcl_meas)            # measurements vector

    # compute Kalman's Gain
    S =  np.linalg.inv(np.dot(np.dot(H,P), H.transpose())+R)
    K = np.dot(np.dot(P,H.transpose()),S)
    
    # compute measurements residuals
    resid = np.array([[Z[0][0]-robot_estimated_pose[0][0]],
                      [Z[1][0]-robot_estimated_pose[1][0]],
                      [Z[2][0]-robot_estimated_pose[2][0]]])

    # update state and covariance estimates
    robot_estimated_pose += np.dot(K,resid)
    P = np.dot(np.eye(n) - np.dot(K,H),P)

def towerNavigation():
    """ 
    Implements navigation to Target towers through the playground
    INPUTS:
    TODO
    """

    global robot_estimated_pose, U_bar

    # define initial e final point when the robot receive the id of the targeted tower
    xd = (robot_estimated_pose[0][0], TOWERS[current_goal][0])
    yd = (robot_estimated_pose[1][0], TOWERS[current_goal][1])

    # define the robot deviation from the required trajectory
    delta_x = xd[1] - xd[0]
    delta_y = yd[1] - yd[0]

    # generates the direction of the motion based on the euclidian distance from goal
    alpha = np.arctan2(delta_y, delta_x)

    # check if the robot is near its goal (this will change in obstacle avoidance behaviour)
    goal_distance = (delta_x**2 + delta_y**2)**0.5
    
    # set near_goal
    near_goal = 0
    if goal_distance < NEAR_GOAL_DISTANCE:
        near_goal = 1

    # SAFETY CHECK: the controller will generates cmd_vel commands only if the safety condition is satisfied
    # if safety condition is satisfied then: enable == 1;
    if is_safe == 1:
        U_bar[0] = MAX_VEL*np.cos(alpha)
        U_bar[1] = MAX_VEL*np.sin(alpha)

    return near_goal

def getRobotPose():
    """
    Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
    x,y and theta.

    OUTPUTS:
        a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
    """
    try:
        tf_listener.waitForTransform('/map','/base_link', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Navigation node: " + str(e))

    # transform from quaternion to euler angles
    euler = tf.transformations.euler_from_quaternion(rot)
    
    return np.array([trans[0], trans[1], euler[2]])   # [xR,yR,theta] 


def scanCallback(msg):
    """
    TODO
    """
    global current_range
    current_range = msg.ranges


def laserScanManager():
    """
    Alters is_safe status
    TODO
    """
    global is_safe

    # check if an obstacle is detected below the proximity threeshold
    proximity = False
    for values in current_range:
        if values < PROXIMITY_THREESHOLD:
            proximity = True
            break

    # perform a safety check
    dontcare_condition = np.array(current_range) < DONTCARE
    new_is_safe = True
    rospy.logwarn("All ones? {}".format(all(dontcare_condition)))
    if proximity and (all(dontcare_condition) == False) and (len(dontcare_condition) != 0):
        new_is_safe = False
    is_safe = new_is_safe

    # Generate sensing areas: rear right, right, front right, front left, left, rear left
    rear_right_sec  = current_range[0:149]
    right_sec       = current_range[150:309]
    front_right_sec = current_range[310:499]
    front_left_sec  = current_range[500:689]
    left_sec        = current_range[690:859]
    rear_left_sec   = current_range[850:999]

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
          

def navigation():
    global U_bar

    # get current robot_velocity (/base_link /vel) in world_frame (/map)
    robot_vel = np.array([[current_vel.linear.x],           # linear x-velocity (robot frame)
                          [current_vel.linear.y],           # linear y-velocity (robot frame)
                          [current_vel.angular.z]])        # angular velocity  (robot frame)

    # G matrix converts from robot to world frame
    G = np.array([[np.cos(robot_estimated_pose[2][0]), -np.sin(robot_estimated_pose[2][0]), 0],
                  [np.sin(robot_estimated_pose[2][0]),  np.cos(robot_estimated_pose[2][0]), 0],
                  [0, 0, 1.]])

    # convert velocity commands from robot to world frame
    robot_world_vel = np.dot(G, robot_vel)
    
    robot_pose = getRobotPose()
    
    # EKF
    holonomicEKFPrediction(robot_pose,U_bar)
    holonomicEKFUpdate(robot_pose)

    # use the global planner (TOWER NAVIGATION)
    near_goal = towerNavigation()

    # use kinematic compensator
    robot_unsmoothed_cmd_vel = kinematic_compensator()
    
    # velocity smoothing
    smoothed_cmd_vel = velocity_smoother(robot_unsmoothed_cmd_vel, robot_vel)

    # updates is_safe and process /scan information
    rear_right, right, front_right, front_left, left, rear_left = (None,None,None,None,None,None)
    if len(current_range) != 0;
        rear_right, right, front_right, front_left, left, rear_left = laserScanManager()

    # Fuzzy
    if not is_safe:
        smoothed_cmd_vel = obstacleAvoider(smoothed_cmd_vel)

    # publish vel commands in /unsafe/cmd_vel topic
    unsafe_msg = Twist()
    unsafe_msg.linear.x = smoothed_cmd_vel[0]
    unsafe_msg.linear.y = smoothed_cmd_vel[1]
    unsafe_msg.angular.z = anglePController()
    pub.publish(unsafe_msg)
    
def main():
    """ The main ros loop"""

    global KP, MAX_DOT_THETA, MAX_ACC, T_MAX, KS, ANGLE_DEADZONE, MAX_VEL, TOWER1, \
        TOWER2, TOWER3, TOWER4, NEAR_GOAL_DISTANCE, TOWERS, PROXIMITY_THREESHOLD, DONTCARE

    #Init node
    rospy.init_node('game_navigation')

    # Subscribers 
    player_info_sub = rospy.Subscriber('/kinect2/player_filtered_info', PlayerInfo, angleCallback)
    game_goal_sub   = rospy.Subscriber('/game/goal', Goal, goalCallback)
    vel_sub   = rospy.Subscriber('/vel', Twist, velCallback)
    laser_sub   = rospy.Subscriber('/scan', LaserScan, scanCallback)

    rate = rospy.Rate(60)
    
    # get param values
    try:
        KP = rospy.get_param("/game_navigation/kp")
        MAX_DOT_THETA = rospy.get_param("/game_navigation/max_dot_theta")
        MAX_ACC = rospy.get_param("/game_navigation/max_acc")
        T_MAX = rospy.get_param("/game_navigation/t_max")
        KS = rospy.get_param("/game_navigation/ks")
        ANGLE_DEADZONE = rospy.get_param("/game_navigation/angle_deadzone")
        MAX_VEL = rospy.get_param("/game_navigation/max_vel")
        TOWER1 = rospy.get_param("/tower_1")
        TOWER2 = rospy.get_param("/tower_2")
        TOWER3 = rospy.get_param("/tower_3")
        TOWER4 = rospy.get_param("/tower_4")
        NEAR_GOAL_DISTANCE = rospy.get_param("/game_navigation/near_goal_distance")
        PROXIMITY_THREESHOLD = rospy.get_param("/game_navigation/proximity_threeshold")
        DONTCARE = rospy.get_param("/game_navigation/dontcare")

    except KeyError as e:
        rospy.logfatal("game_navigation node: Param error! Check 'navigation.yaml'. >> " + str(e))

    # init variables
    robot_init_pose = getRobotPose()
    robot_estimated_pose = np.array([robot_init_pose[0],robot_init_pose[1],robot_init_pose[2],0.,0.,0.])
    TOWERS = (TOWER1,TOWER2,TOWER3,TOWER4)
    
    while not rospy.is_shutdown():
        navigation()
        rate.sleep()
    
    #rospy.spin()

if __name__ == '__main__':
    main()

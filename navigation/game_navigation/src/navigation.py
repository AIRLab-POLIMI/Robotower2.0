#!/usr/bin/env python
from behavior_manager.msg import Goal
from kinect_tracker.msg import PlayerInfo
from geometry_msgs.msg import Twist
import numpy as np
import rospy
import copy
import tf

# TODO put all parameter in a config.yaml file. Read using rosparam get. 
KP = 4;                 # Proportional Gain (anglePControllerCallback())
MAX_DOT_THETA = 2.5     # Maximum allowed angular velocity (in rad/sec)
ANGLE_DEADZONE = 0.2    # DeadZone range (in rad/sec), values in this range are considered zero

current_vel = None
current_goal = None
current_angle_diff = None

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
    current_goal = msg.tower_number

def angleCallback(msg):
    """
    Updates current camera off-center player angle
    """
    global current_angle_diff
    current_angle_diff = msg.angle

def anglePController(theta_diff):
    """
    The proportional controller callback to adjust robot 
    orientation in order to track the human player

    Input:
        @theta_diff : the main camera (kinect) off-center player angle.
    """

    # Dead zone (Jerk-smother) used in order to eliminate angular
    # jerking while tracking
    
    if abs(theta_diff) < ANGLE_DEADZONE:
        theta_diff = 0

    # Proportional Controller
    dot_theta = KP*theta_diff

    # Angular velocity clamping (max angular velocity in rad/sec)
    if dot_theta >= MAX_DOT_THETA:
        return MAX_DOT_THETA
    elif dot_theta <= -MAX_DOT_THETA:
        return -MAX_DOT_THETA
    else:
        return dot_theta

def velocity_smoother(bar_u1R, bar_u2R, dot_xRm, dot_yRm, is_safe):
    """
    WRITE ME

    Input:
        @ bar_u1: unsmoothed x-cmd_vel (robot frame)
        @ bar_u2: unsmoothed y-cmd_vel (robot frame)
        @ dot_xRm : robot x-vel (robot frame)
        @ dot_yRm : robot y-vel (robot frame)
        @ is_safe : avoider control variable
    """
    max_acc = 0.5  # maximum allowed acceleration
    max_vel = 0.7  #(rosparam_get max_vel)
    T_max = 3.
    k = 50.

    if is_safe == True:
        initial_vel_x = dot_xRm
        initial_vel_y = dot_yRm
    else:
        initial_vel_x = 0
        initial_vel_y = 0

    # define acceleration
    initial_acc_x = bar_u1R - initial_vel_x
    initial_acc_y = bar_u2R - initial_vel_y

    # X-accelerations clamping
    if initial_acc_x >= max_acc:
        initial_acc_x = max_acc
    elif initial_acc_x <= -max_acc:
        initial_acc_x = -max_acc

    # Y-accelerations clamping
    if initial_acc_y >= max_acc:
        initial_acc_y = max_acc
    elif initial_acc_y <= -max_acc:
        initial_acc_y = -max_acc

    # generate interpolating polynomia and cmd_vel
    t1 = abs(1 / k * (T_max) / (max_vel))
    t2 = abs(1 / k * (T_max) / (max_vel))
    x1 = t1
    x2 = t2
    # smoothed cmd_vel
    cmd_x = initial_vel_x + initial_acc_x * x1
    cmd_y = initial_vel_y + initial_acc_y * x2
    return cmd_x, cmd_y


def kinematic_compensator(bar_u1, bar_u2, bar_u3, theta_hat):
    """
    WRITE ME
    """
    # G matrix converts from robot to world frame
    G = np.array([[np.cos(theta_hat), -np.sin(theta_hat), 0],
                  [np.sin(theta_hat),  np.cos(theta_hat), 0],
                  [0, 0, 1.]])
    # iG matrix converts from world to robot frame
    iG = np.linalg.inv(G)
    # define vector of velocity commands
    bar_u = np.array([[bar_u1],
                      [bar_u2],
                      [bar_u3]])
    # convert velocity commands from world to robot frame
    bar_uR = np.dot(iG, bar_u)
    bar_u1R = bar_uR[1]
    bar_u2R = bar_uR[2]
    bar_u3R = bar_uR[3]
    # output the velocity command in robot frame
    return bar_u1R, bar_u2R, bar_u3R


def holonomic_EKF(xR, yR, theta, bar_u1, bar_u2, bar_u3):
    """
    WRITE ME
    """
    # [xR, yR, theta] is the robot pose coming from AMCL
    xhat0 = np.array([[xR],           #observed state vector
                      [yR],
                      [theta],
                      [0],              #initial x-velocity
                      [0],              #initial y-velocity
                      [0]])             #initial angular-velocity
    n = 6                            #number of states
    q = 0.6                          #std of the process
    r1 = 0.5                         #std of position measurement
    r2 = np.pi/36                    #std of orientation measurement
    P = np.eye(n)                    #initial covariance (6x6 matrix)
    Q = (q**2)*np.eye(n)             #Noise covariance matrix of the process (6x6 matrix)
    R = np.array([r1**2, 0, 0],      #Covariance of measurements
                 [0, r1**2, 0],
                 [0, 0, r2**2])
    T = 0.02                         #sampling time

    # Observation Matrix and measurements vector
    H = np.array([[1., 0, 0, 0, 0, 0],
                  [0, 1., 0, 0, 0, 0],
                  [0, 0, 1., 0, 0, 0]])
    amcl_meas = np.array([[xR],
                          [yR],
                          [theta],
                          [0],
                          [0],
                          [0]])
    Z = np.dot(H, amcl_meas)

    # Transition Matrix (theta is in rad)
    chi_x = (-np.sin(theta) / (2 * np.sin(np.pi / 3)) + (np.cos(theta) / (2 + 2 * np.cos(np.pi / 3))))
    chi_y = (-np.cos(theta) / (2 * np.sin(np.pi / 3)) + (np.sin(theta) / (2 + 2 * np.cos(np.pi / 3))))
    F = np.array([1., 0, 0, chi_x*T, 0, 0],
                 [0, 1., 0, 0, chi_y*T, 0],
                 [0, 0, 1., 0, 0, T],
                 [0, 0, 0, 1., 0, 0],
                 [0, 0, 0, 0, 1., 0],
                 [0, 0, 0, 0, 0, 1.])

    # Control Matrix and Control inputs
    B = np.array([0, 0, 0],
                 [0, 0, 0],
                 [0, 0, 0],
                 [1., 0, 0],
                 [0, 1., 0],
                 [0, 0, 1.])
    bar_u = np.array([[bar_u1],
                      [bar_u2],
                      [bar_u3]])

    # A priori prediction
    xhat = np.dot(F, xhat0)+np.dot(B, bar_u)
    P = np.dot(F, P, F.transpose())

def obstacle_avoider():
    """Implements obstacle avoidance using Fuzzy logic"""
    pass

def navigation():
    """ Implements the navigation"""

    # get current robot_velocity (/base_link /vel) in world_frame (/map)

    # get robot pose
    try:
        tf_listener.waitForTransform('/map','/base_link', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Navigation node: " + str(e))

    # use EKF


    # use the global planner (TOWER NAVIGATION)


    # use kinematic compensator

    # velocity smoothing

    unsafe_msg = Twist()
    #unsafe_msg.linear.x = 2

    pub.publish(unsafe_msg)
    
def main():
    """ The main ros loop"""

    #Init node
    rospy.init_node('game_navigation')

    # Subscribers 
    player_info_sub = rospy.Subscriber('kinect2/player_filtered_info', PlayerInfo, angleCallback)
    game_goal_sub   = rospy.Subscriber('game/goal', Goal, goalCallback)
    vel_sub   = rospy.Subscriber('/vel', Twist, velCallback)

    rate = rospy.Rate(60)
    
    while not rospy.is_shutdown():
        navigation()
        rate.sleep()

    #rospy.spin()

if __name__ == '__main__':
    main()

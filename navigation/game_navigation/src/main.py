#!/usr/bin/env python
from behavior_manager.msg import Goal
from kinect_tracker.msg import PlayerInfo
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from navigation import Navigation
import numpy as np
import rospy
import copy
import tf
    
def main():
    """ The main ros loop"""
    #Init node
    rospy.init_node('game_navigation')

    rate = rospy.Rate(60)
    navigation = None    # init navigation module variable
    
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
        RR_LOWER_BOUND  = rospy.get_param("/game_navigation/rr_lower_bound")
        R_LOWER_BOUND = rospy.get_param("/game_navigation/r_lower_bound")
        FR_LOWER_BOUND = rospy.get_param("/game_navigation/fr_lower_bound")
        FL_LOWER_BOUND = rospy.get_param("/game_navigation/fl_lower_bound")
        L_LOWER_BOUND = rospy.get_param("/game_navigation/l_lower_bound")
        RL_LOWER_BOUND = rospy.get_param("/game_navigation/rl_lower_bound")

        navigation = Navigation(KP, MAX_DOT_THETA, MAX_ACC, T_MAX, KS, ANGLE_DEADZONE, MAX_VEL, TOWER1, TOWER2, TOWER3, TOWER4, NEAR_GOAL_DISTANCE,
                    PROXIMITY_THREESHOLD, DONTCARE, RR_LOWER_BOUND, R_LOWER_BOUND, FL_LOWER_BOUND, FL_LOWER_BOUND, L_LOWER_BOUND, RL_LOWER_BOUND)

    except KeyError as e:
        rospy.logfatal("game_navigation node: Param error! Check 'navigation.yaml'. >> " + str(e))
        exit(-1)

    # Subscribers 
    player_dist_sub = rospy.Subscriber('/kinect2/player_info',PlayerInfo, navigation.playerInfoCallback)
    player_info_sub = rospy.Subscriber('/kinect2/player_filtered_info', PlayerInfo, navigation.angleCallback)
    game_goal_sub   = rospy.Subscriber('/game/goal', Goal, navigation.goalCallback)
    vel_sub   = rospy.Subscriber('/vel', Twist, navigation.velCallback)
    laser_sub   = rospy.Subscriber('/scan', LaserScan, navigation.scanCallback)

    # Publishers
    pub = rospy.Publisher('unsafe/cmd_vel', Twist, queue_size=1)

    while not rospy.is_shutdown():
        # publish vel commands in /unsafe/cmd_vel topic
        pub.publish(navigation.navigate())
        rate.sleep()
    
    #rospy.spin()

if __name__ == '__main__':
    main()
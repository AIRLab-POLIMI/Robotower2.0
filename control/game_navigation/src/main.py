#!/usr/bin/env python
from behavior_control.msg import Goal
from kinect_tracker.msg import PlayerInfo
from geometry_msgs.msg import Twist
from player_tracker.msg import TowerArray
from sensor_msgs.msg import LaserScan
from navigation import Navigation
from std_msgs.msg import Float32
from geometry_msgs.msg import PolygonStamped
import numpy as np
import rospy
import copy
import tf
    
# TODO: Change nomenclature of MAX_VEL param, which is better to be called MAX_SPEED.

def main():
    """ The main ros loop"""
    #Init node
    rospy.init_node('game_navigation')

    rate = rospy.Rate(60)
    navigation = None    # init navigation module variable

    difficulties = None  # init the difficulty control variable
    
    # get param values
    try:
        difficulties = rospy.get_param("/difficulties")
        
        KP = rospy.get_param("/game_navigation/kp")
        MAX_DOT_THETA = rospy.get_param("/game_navigation/max_dot_theta")
        KS = rospy.get_param("/game_navigation/ks")
        ANGLE_DEADZONE = rospy.get_param("/game_navigation/angle_deadzone")

        MAX_VEL = difficulties[rospy.get_param('/current_difficulty')]['max_speed']
        MAX_ACC = difficulties[rospy.get_param('/current_difficulty')]['max_acc']
        T_MAX = difficulties[rospy.get_param('/current_difficulty')]['ks']

        TOWER1 = rospy.get_param("/tower_1")
        TOWER2 = rospy.get_param("/tower_2")
        TOWER3 = rospy.get_param("/tower_3")
        TOWER4 = rospy.get_param("/tower_4")

        NEAR_GOAL_DISTANCE = rospy.get_param("/game_navigation/near_goal_distance")
        PROXIMITY_THREESHOLD = rospy.get_param("/game_navigation/proximity_threeshold")
        
        DONTCARE = rospy.get_param("/game_navigation/dontcare")
        RR_LOWER_BOUND = rospy.get_param("/game_navigation/rr_lower_bound")
        R_LOWER_BOUND  = rospy.get_param("/game_navigation/r_lower_bound")
        FR_LOWER_BOUND = rospy.get_param("/game_navigation/fr_lower_bound")
        FL_LOWER_BOUND = rospy.get_param("/game_navigation/fl_lower_bound")
        L_LOWER_BOUND  = rospy.get_param("/game_navigation/l_lower_bound")
        RL_LOWER_BOUND = rospy.get_param("/game_navigation/rl_lower_bound")

        navigation = Navigation(KP, MAX_DOT_THETA, MAX_ACC, T_MAX, KS, ANGLE_DEADZONE, MAX_VEL, TOWER1, TOWER2, TOWER3, TOWER4, NEAR_GOAL_DISTANCE,
                    PROXIMITY_THREESHOLD, DONTCARE, RR_LOWER_BOUND, R_LOWER_BOUND, FL_LOWER_BOUND, FL_LOWER_BOUND, L_LOWER_BOUND, RL_LOWER_BOUND)

    except KeyError as e:
        rospy.logfatal("game_navigation node: Param error! Check 'navigation.yaml'. >> " + str(e))
        exit(-1)

    # Subscribers 
    #player_dist_sub = rospy.Subscriber('/kinect2/player_filtered_info',PlayerInfo, navigation.playerInfoCallback)
    #player_info_sub = rospy.Subscriber('/playground_center', Float32, navigation.angleCallback)
    game_goal_sub   = rospy.Subscriber('/game/goal', Goal, navigation.goalCallback)
    vel_sub         = rospy.Subscriber('/vel', Twist, navigation.velCallback)
    laser_sub       = rospy.Subscriber('/scan', LaserScan, navigation.scanCallback)
    laser_tower_sub = rospy.Subscriber('/estimated_tower_positions', TowerArray, navigation.tpos_callback)
    tower_rectangle_sub = rospy.Subscriber('/tower_rectangle', PolygonStamped, navigation.tower_rectangle_callback)

    # TODO adjust this subscriber: move to another location, ask Ewerton
    angle_sub = rospy.Subscriber('/angle', Float32, navigation.angleCallback)
 
    # Publishers
    pub = rospy.Publisher('unsafe/cmd_vel', Twist, queue_size=1)

    previous_diff = rospy.get_param('/current_difficulty')


    while not rospy.is_shutdown():

        cur_difficulty = rospy.get_param('/current_difficulty')
        if cur_difficulty != previous_diff:
            navigation.set_max_speed(difficulties[cur_difficulty]['max_speed'])
            navigation.set_max_acc(difficulties[cur_difficulty]['max_acc'])
            navigation.set_ks(difficulties[cur_difficulty]['ks'])

        previous_diff = cur_difficulty

        # publish vel commands in /unsafe/cmd_vel topic
        pub.publish(navigation.navigate())
        rate.sleep()
    
    #rospy.spin()

if __name__ == '__main__':
    main()

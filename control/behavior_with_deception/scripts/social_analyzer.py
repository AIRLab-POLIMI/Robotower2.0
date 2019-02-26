#!/usr/bin/env python

import rospy
import tf

import numpy
import random
import math
import time

import target_finder
import outcome_matrices_creator
import interdependence
import correspondence
import deception

from game_manager.msg import Towers, TowerState

from behavior_with_deception.srv import DeceptiveCommand

from behavior_with_deception.msg import Goal
from behavior_with_deception.msg import Deception
from behavior_with_deception.msg import Idle
from behavior_with_deception.msg import DeceptionCommandMsg

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy

class SocialAnalyzer(object):
    def __init__(self, percentage_deception):
        
        # Initializing towers' positions and agents' positions
        self.player_xy = self.robot_xy = None
        self.towers_xy = numpy.zeros(4)

        # Target array says for each tower how 'likely' the robot/player wants to win it
        # Target_AGENT says which tower is the chosen one (not really used for the player, it could be implemented)
        self.target_array_robot = self.target_array_player = numpy.zeros(4)
        self.won_towers = self.target_robot = self.target_player = [1, 1, 1, 1]

         

        # Outcome matrices for agents
        self.outcome_matrix_robot = self.outcome_matrix_player = numpy.zeros((4,4))

        # Mapping outcome matrices into social situation space
        # Alpha --> interdependence
        # Beta --> correspondence
        # Check correspondence.py and interdependency.py for more info
        self.alpha_r = 0
        self.beta = 0

        self.count = 0

        self.target = 0

        # Variables needed for managing the deception
        # self.deception_activated = False
        self.deception_done = False
        # self.percentage_deception = 0.5 # for difficult levels
        self.base_percentage_deception = percentage_deception # Base percentage useful for recovery
        self.percentage_deception = percentage_deception# rospy.get_param("percentage_deception")

        # There are three types of deception, see manual for more info
        self.type_deception = 0

        # Fake target is used for moving the robot to the false direction, it is calculated using the outcome matrices
        self.fake_target = 0
        self.real_target = 0

        # For deception number 3.
        # self.num_slopes = 0
        # self.current_slopes = 0
        # self.deception_duration_time_per_slope = 100

        self.go_to_center = False
        self.idle = False
    
        self.tf_listener = tf.TransformListener()
        self.sub = rospy.Subscriber('/player_filtered', PointStamped, self.callback_player_filtered)
        self.sub_towers = rospy.Subscriber("game_manager/towers/State", Towers, self.tower_callback)
        self.sub_joy = rospy.Subscriber("/joy", Joy, self.joy_callback, queue_size=1)
        self.pub = rospy.Publisher('/game/goal', Goal, queue_size=1)
        self.pub_deception = rospy.Publisher('/game/is_deceiving', Deception, queue_size=1)
        self.pub_idle = rospy.Publisher('/game/is_idle', Idle, queue_size=1)
        
        self.steering_mode = rospy.get_param('/steering_mode')
        
        if(self.steering_mode):
            self.pub_deception_command = rospy.Publisher('/action/deception_command', DeceptionCommandMsg, queue_size=1)
        else:
            rospy.wait_for_service('/behavior_with_deception/make_deception')
            
            try:
                self.srv_handler = rospy.ServiceProxy('/behavior_with_deception/make_deception', DeceptiveCommand)
            except rospy.ServiceException, e:
                rospy.loginfo( "Service call failed: %s,"%e)	

        # Service for going to the center of the playground

        # rospy.wait_for_service('/behavior_with_deception/go_center_playground')     
        # try:
        #     self.srv_handler = rospy.ServiceProxy('/behavior_with_deception/go_center_playground', GoToCenterPlayground)
        # except rospy.ServiceException, e:
        #     rospy.loginfo( "Service call failed: %s,"%e)

    def joy_callback(self, joy_msg):
        if(joy_msg.buttons[5] == 1):
            # Pressing deadman's button
            if(joy_msg.buttons[1]==1):
                # Pressing button 2
                rospy.logwarn("No deception")
                self.percentage_deception = 1
            elif(joy_msg.buttons[3]==1):
                rospy.logwarn("Deception")
                self.percentage_deception = self.base_percentage_deception

    def tower_callback(self, tower_msg):
        '''Callback for TowerState for Laura Cativa!
        The callback checks if a tower has been captured by the playe. If yes, the tower can not be considered as possible target.
        '''
        
        if tower_msg.tw1.status == TowerState.TYPE_TOWER_CAPTURED:
            self.target_robot[0] = 0
            self.target_player[0] = 0
            self.won_towers[0] = 0
        else:
            self.won_towers[0] = 1

        
        if tower_msg.tw2.status == TowerState.TYPE_TOWER_CAPTURED:
            self.target_robot[1] = 0
            self.target_player[1] = 0
            self.won_towers[1] = 0
        else:
            self.won_towers[1] = 1
        
        if tower_msg.tw3.status == TowerState.TYPE_TOWER_CAPTURED:
            self.target_robot[2] = 0
            self.target_player[2] = 0
            self.won_towers[2] = 0
        else:
            self.won_towers[2] = 1

        if tower_msg.tw4.status == TowerState.TYPE_TOWER_CAPTURED:
            self.target_robot[3] = 0
            self.target_player[3] = 0
            self.won_towers[3] = 0
        else:
            self.won_towers[3] = 1
        

    def callback_player_filtered(self, msg):
        # msg = self.transform_player_pose(msg)
        if msg is not None:
            self.player_xy = numpy.array([msg.point.x, msg.point.y])

    def talk_to_service(self):
        """Call the behavior_service

        Returns
            resp
        """
        if(self.steering_mode):
            #if(self.type_deception != 3):
            deceptive_command = DeceptionCommandMsg()
            deceptive_command.real_target = self.real_target
            deceptive_command.fake_target = self.fake_target
            deceptive_command.type = self.type_deception
            self.pub_deception_command.publish(deceptive_command)
        elif self.go_to_center:
            try:
                resp = self.srv_handler(-1, -1, -1)
                return resp
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        elif self.idle:
            try:
                resp = self.srv_handler(-2, -2, -2)
                return resp
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
        else:
            try:
                resp = self.srv_handler(self.real_target, self.fake_target, self.type_deception)
                return resp
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def get_pose(self, source, target):
        """
        Gets robot global position. That is, performs a TF transformation from /base_link to /player_filtered_link and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform( target, source, rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform( target, source, rospy.Time(0))

            return numpy.array([trans[0], trans[1]])   # [xR,yR,theta]

        except Exception as e:
            rospy.logerr("Behavior with deception node: " + str(e))       



    def transform_player_pose(self,msg):
        '''Transforming player position from base_link to map '''
        try:
            return self.tf_listener.transformPoint('/map', msg)
        except Exception as e:
            rospy.logerr('Social analizer: {}'.format(str(e)))
        
    def set_position_agents(self):
        ''' Getting agents' position '''
        self.robot_xy = numpy.zeros(2)
        # self.player_xy = self.get_pose("/player_filtered_link", "/base_link")
        self.towers_xy = numpy.array([self.get_pose("/tower_1", "/base_link"), self.get_pose("/tower_2", "/base_link"),
                            self.get_pose("/tower_3", "/base_link"), self.get_pose("/tower_4", "/base_link")])

  

    def check_incoming_messages(self):
        ''' Check if the positions received of the agents is not null'''
        if (not self.player_xy is None and not self.towers_xy is None):
            return True
        else:
            return False  

    def random_idle_continue_center(self,previous_target):
        '''
        This function is used for decreasing the difficulty when playing.
        In a non deterministic way, the funcion adds idle behavior or orders the robot6 to position at the center of the playground.
        There are 3 options:
        1. Continue the normal approach, that means going to the next tower;
        2. Stay idle for a couple of seconds
        3. Go to the center of the playground and starts again the algorithm
        '''
        if not self.target == previous_target:
            rospy.loginfo('previous: {} new:{}'.format(previous_target, self.target))
            if(self.steering_mode):
                option=1
            else:
                option = random.randint(1, 5)
            if option == 1 or option == 4 :
                rospy.loginfo("CONTINUE")
                return
            elif option==2:
                rospy.loginfo("IDLE")
                self.idle = True
                self.publish_idle(True)
                self.publish_deception(True)
                self.talk_to_service()
                self.publish_idle(False)
                self.publish_deception(False)
                self.idle = False
                return
            else:
                rospy.loginfo("CENTER")
                self.go_to_center = True
                self.talk_to_service()
                self.get_targets()
                return

    
    def update_won_towers(self):
        for i in range(4):
            if self.won_towers[i] == 0:
                self.target_robot[i] = 0
                self.target_player[i] = 0
            else:
                self.target_robot[i] = 1
                self.target_player[i] = 1


    def get_targets(self):
        # Calculating how likely to win a tower for the robot
        self.target_array_robot = target_finder.target_preferences_robot(self.player_xy, self.robot_xy, self.towers_xy,
                                                                        self.target_robot)

        previous_target = self.target
        self.update_won_towers()
        # Checking which tower is the target
        self.target = target_finder.find_target(self.target_array_robot)

        if not self.go_to_center:
            self.random_idle_continue_center(previous_target)

            # To increase the outcome of the target once chosen
            # (for not changing mind during an action)
            # (action is defined as choosing the tower and all the procedure for winning it)
            self.target_robot[self.target] = 1.3
            self.target_array_player = target_finder.target_preferences_player(self.player_xy, self.robot_xy,
                                                                            self.towers_xy,
                                                                            self.target_player)
            self.target_player[target_finder.find_target(self.target_array_player)] = 1
           
        else:
            self.go_to_center = False

        # rospy.loginfo('robot: {}  player:{}'.format(self.target_array_robot, self.target_array_player))

    def get_outcome_matrices(self):
        # Creating the outcome matrices
        self.outcome_matrix_robot = outcome_matrices_creator.outcome_matrix_robot(self.target_array_robot,
                                                                                self.towers_xy,
                                                                                self.player_xy, self.robot_xy)

        self.outcome_matrix_player = outcome_matrices_creator.outcome_matrix_player(self.target_array_player,
                                                                                    self.towers_xy,
                                                                                    self.robot_xy)

    def map_outcome_matrices_in_interdependence_space(self):
        # Mapping the outcome matrices to the social situation space
        self.beta = correspondence.correspondence(self.outcome_matrix_robot, self.outcome_matrix_player)
        self.alpha_r = interdependence.interdependence(self.outcome_matrix_robot, self.target_array_robot)


    

    def check_for_deception(self):
        if self.beta < -0.6 and self.alpha_r > 0.6:
            
            if random.randint(1, 100)/100 > self.percentage_deception : # put 2 to avoid deception
                self.real_target = self.target
                self.fake_target = deception.get_fake_target(self.outcome_matrix_robot, self.outcome_matrix_player, self.real_target)
                self.type_deception = deception.get_type_deception(self.fake_target, self.real_target, self.robot_xy, self.player_xy)
                
                rospy.loginfo('type:{}'.format(self.type_deception))
                if not self.type_deception == 4:
                    rospy.loginfo("Outcome matrix robot: {}".format(self.outcome_matrix_robot))
                    if(not self.steering_mode):
                        # Laura's case
                        self.publish_deception(True)
                        rospy.loginfo('Deception activated type:{}  fake target:{}  real target:{}'.format(self.type_deception, self.fake_target, self.real_target))
                        self.talk_to_service()
                        self.deception_done = True
                        rospy.loginfo('Service finished')
                    else:
                        rospy.loginfo('Deception activated type:{}  fake target:{}  real target:{}'.format(self.type_deception, self.fake_target, self.real_target))
                        self.talk_to_service()
                        self.publish_deception(True)
                        self.deception_done = True
                        rospy.loginfo('Service finished')
                    return
        
        self.publish_target(self.target)
        self.publish_deception(False)
        

        # For checking if deception should be stopped
        # if not self.real_target == self.target:
        #     self.deception_done = False
        #     self.deception_activated = False
        #     self.current_slope = 0

    

    def publish_target(self, target):
        new_goal = Goal()
        new_goal.header.stamp = rospy.get_rostime()
        new_goal.tower_number = target+1
        self.pub.publish(new_goal)

    def publish_idle(self, idle):
        new_idle_msg = Idle()
        new_idle_msg.header.stamp = rospy.get_rostime()
        new_idle_msg.being_idle = idle
        self.pub_idle.publish(new_idle_msg)

    def publish_deception(self, message):
        new_deception_msg = Deception()
        new_deception_msg.header.stamp = rospy.get_rostime()
        new_deception_msg.being_deceptive = message
        self.pub_deception.publish(new_deception_msg)




    # def transform_positions(self):
    #     tower2_xy = numpy.array([0,0])
    #     tower3_xy = numpy.array([3.3,0])
    #     distance_t2_robot = target_finder.distance(self.robot_xy, self.towers_xy[1,:])
    #     distance_t3_robot = target_finder.distance(self.robot_xy, self.towers_xy[2,:])
    #     distance_t2_player = target_finder.distance(self.player_xy, self.towers_xy[1,:])
    #     distance_t3_player = target_finder.distance(self.player_xy, self.towers_xy[2,:])
        
    #     rospy.loginfo('t2_player: {}  t2_robot:{}'.format(distance_t2_player, distance_t2_robot))
    #     rospy.loginfo('t1_player: {}  t1_robot:{}'.format(target_finder.distance(self.player_xy, self.towers_xy[0,:]), target_finder.distance(self.robot_xy, self.towers_xy[0,:])))

    #     robot_y = (distance_t3_robot ** 2 - distance_t2_robot ** 2 - 3.3 ** 2) / (-6.6)
    #     robot_x = math.sqrt(distance_t2_robot ** 2 - robot_y ** 2)

    #     self.robot_xy = numpy.array([robot_y, robot_y])


    #     player_y = (distance_t3_player ** 2 - distance_t2_player ** 2 - 3.3 ** 2) / (-6.6)
    #     try:
    #         player_x = math.sqrt(distance_t2_player ** 2 - player_y ** 2)  
    #     except expression as identifier:
    #         player_x = 0 
            
    #     self.player_xy = numpy.array([player_x, player_y])

    #     self.towers_xy = numpy.array([[3.3, 0], [0, 0], [0, 3.3], [3.3, 3.3]])

    # def get_last_deception(self):
    #     if self.deception_done and not self.real_target == self.target:
    #         self.deception_done = False
    #     return self.deception_done

    # def is_deceiving(self):
    #     if self.deception_activated:
    #         if (self.count < self.deception_duration_time_per_slope and not self.type_deception == 3 and not self.type_deception == 4):
    #             self.count += 1
    #             # self.publish(self.fake_target)
    #             rospy.loginfo('Doing deception, type: {}  target: {}'.format(self.type_deception, self.real_target))
    #         else:
    #             if (self.type_deception == 3 and self.count < self.deception_duration_time_per_slope*self.num_slopes):
    #                 rospy.loginfo('Doing deception, type: {}  real target: {}  fake target : {}'.format(self.type_deception, self.real_target, self.fake_target))
    #                 self.count += 1
    #                 if self.count % self.deception_duration_time_per_slope == 0:
    #                     self.current_slopes += 1
    #                 if self.current_slopes % 2 == 0:
    #                     self.publish_target(self.fake_target)
    #                 else:
    #                     self.publish_target(self.real_target)
    #             else:
    #                 self.deception_activated = False
    #                 self.deception_done = True
    #     else:
    #         self.count = 0
    #         self.deception_done = False
    #         self.deception_activated = False
    #         self.publish_target(self.target)
        
    #     self.publish_deception(self.deception_activated)

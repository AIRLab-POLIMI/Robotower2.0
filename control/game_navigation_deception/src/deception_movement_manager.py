#!/usr/bin/env python

import rospy
import tf

import numpy
import random
import math

from behavior_control.msg import Goal

class DeceptionMovementManager(object):
    
    def __init__(self):

        self.fake_target = self.real_target = 0
        self.fake_target_coordinates = self.real_target_coordinates = self.robot_coordinates = numpy.zeros(2)

        self.traslations_xy = numpy.zeros(2)

        self.type_deception = -1

        self.distance_between_two_straight_steps = 0.40

        self.deception = False
    
    def set_targets(self, real_target, fake_target):
        self.fake_target = fake_target
        self.real_target = real_target

    def set_type_deception(self, type_dec):
        self.type_deception = type_dec

    def set_targets_coordinates(self, towers_coordinates):
        self.fake_target_coordinates = towers_coordinates[self.fake_target]
        self.real_target_coordinates = towers_coordinates[self.real_target]

    def set_robot_coordinates(self, robot_coordinates):
        self.robot_coordinates = robot_coordinates

    def set_traslation_robot_playground(self,traslations_xy):
        self.traslations_xy = traslations_xy

    def trajectory_planner(self):
        '''
        Trajectory planner calculates the trajectory based on the type of deception detected
        '''
        self.check_type_deception()

        rospy.loginfo("Deception type:{} fake target:{} real_target:{}".format(self.type_deception, self.fake_target, self.real_target))

        if self.type_deception == 1 or self.type_deception == 2:
            
            points_trajectory_array = self.trajectory_array_generator_deception_1()

        else:
            if self.type_deception == 3:

                points_trajectory_array = self.trajectory_array_generator_deception_3_1slope()

        return points_trajectory_array
        
    def trajectory_array_generator_deception_1(self):
        '''
        Generator of the trajectory for deception 1 or 2
        '''
        middle_point_targets = (self.fake_target_coordinates + self.real_target_coordinates) / 2
        distance_robot_middle_point = self.distance(middle_point_targets, self.robot_coordinates)

        points_trajectory_array = []
        point = (middle_point_targets + self.robot_coordinates) / 2
        points_trajectory_array.append(point)
        
        if distance_robot_middle_point/4 < 1:
            point = (point + self.fake_target_coordinates) / 2
            points_trajectory_array.append(point)

        points_trajectory_array.append(self.real_target_coordinates)
        
        return points_trajectory_array
        
    def trajectory_array_generator_deception_3_1slope(self):

        distance_robot_faket = self.distance(self.fake_target_coordinates, self.robot_coordinates)

        rospy.loginfo('Robot coordinates:{} Fake_T_Coordinates:{}'.format(self.robot_coordinates, self.fake_target_coordinates))

        points_trajectory_array = []

        point = (self.fake_target_coordinates + self.robot_coordinates) / 2
        points_trajectory_array.append((point + self.robot_coordinates) / 2)
        points_trajectory_array.append(point)

        if distance_robot_faket/4 < 1:
            point = (self.fake_target_coordinates + points_trajectory_array[0]) / 2
            points_trajectory_array.append(point)

        points_trajectory_array.append(self.real_target_coordinates)
        
        return points_trajectory_array

    def distance(self, point1, point2):
        dist = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return dist

    def check_type_deception(self):
        if self.type_deception == 1:
            if -0.1 <= self.traslations_xy[1]*math.sin(0.17) <= 0.1 and self.check:
                self.type_deception = 2
            else :
                if (not 1.5 <= self.robot_coordinates[0]*math.cos(0.17) <= 1.7) and (not self.check):
                    self.type_deception = 3

    def check(self):
        if (fake_target == 0 and real_target == 1) or (fake_target == 2 and real_target == 3) or (fake_target == 1 and real_target == 0) or (fake_target == 3 and real_target == 2):
            return True # Moving on x 
        else:
            return False # Moving on y
            
    # def trajectory_array_generator_deception_3_5slopes(self, movement):

    #     points_trajectory_array = self.trajectory_array_generator_deception_3_3slopes(movement)
    #     points_trajectory_array = numpy.append(points_trajectory_array, self.trajectory_array_generator_deception_3_1slope(movement), axis = 0)
    #     return points_trajectory_array
        

    # def get_parameters_line(self):
    #     if (self.fake_target_coordinates[0] - self.real_target_coordinates[0]) == 0:
    #         type_line = 1 #line of the type x = value
    #         return [type_line, None]
    #     else:
    #         if (self.fake_target_coordinates[1] - self.real_target_coordinates[1]) == 0:
    #             type_line = 2 #line of the type y = value
    #             return [type_line, None]
    #         else:
    #             m = (self.fake_target_coordinates[1] - self.real_target_coordinates[1]) / (self.fake_target_coordinates[0] - self.real_target_coordinates[0])
    #             q = - self.real_target_coordinates[0] * m + self.fake_target_coordinates[1]
    #             return [m,q]

    # def get_segment(self, m, q):
    #     if not q is None:
    #         x = q / (m + 1 / m)
    #         y = - x / m
    #     else:
    #         if m == 1:
    #             y = self.fake_target_coordinates[1]
    #             x = self.robot_coordinates[0] * math.cos(0.17)
    #         else:
    #             y = self.robot_coordinates[1] * math.sin(0.17)
    #             x = self.fake_target_coordinates[0]
    #     return [x * math.cos(0.17), y * math.sin(0.17)]

        
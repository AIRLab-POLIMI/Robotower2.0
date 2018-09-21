#!/usr/bin/env python
import rospy
import tf

import numpy
import random
import math

from behavior_control.msg import Goal


class DeceptionMovementManager(object):
    
    def __init__(self):

        self.fake_target = self.real_target = -1
        self.fake_target_coordinates = self.real_target_coordinates = numpy.zeros(2)
        self.robot_coordinates = numpy.zeros(3)

        self.type_deception = -1

        self.distance_between_two_straight_steps = 0.66

        self.deception = False
    
    def set_targets(self, real_target, fake_target):
        self.fake_target = fake_target
        self.real_target = real_target

    def set_type_deception(self, type_dec):
        self.type_deception = type_dec

    def set_targets_coordinates(self, towers_coordinates):
        self.fake_target_coordinates = towers_coordinates[self.fake_target-1]
        self.real_target_coordinates = towers_coordinates[self.real_target-1]
    
    def set_robot_coordinates(self, robot_coordinates):
        self.robot_coordinates = robot_coordinates

    def trajectory_planner(self):
        parameters_line = self.get_parameters_line()
        final_point_coordinates = self.get_segments(parameters_line[0], parameters_line[1])
        distance_robot_final_point = self.distance(final_point_coordinates, [self.robot_coordinates[0], self.robot_coordinates[1]])

        self.check_type_deception()  #TODO check the half field map values

        if self.type_deception == 1 or self.type_deception == 2:
            
            number_straight_steps = int(distance_robot_final_point / self.distance_between_two_straight_steps)
            
            if number_straight_steps > 2:
                actual_straight_steps = number_straight_steps - random.randint(1,2)
            else:
                actual_straight_steps = 0
            
            points_trajectory_array = self.trajectory_array_generator_deception_1(final_point_coordinates, actual_straight_steps)
        
        else:
            if self.type_deception == 3:

                if (final_point_coordinates[0] - self.real_target_coordinates[0]) < 0 or (final_point_coordinates[1] - self.real_target_coordinates[1]) < 0 :
                    movement = 1 # Going to the right
                else:
                    movement = 2 # Going to the left

                if distance_robot_final_point <= 1.1:
                    points_trajectory_array = self.trajectory_array_generator_deception_3_1slope(movement)

                else:
                    if distance_robot_final_point <= 2.2:
                        points_trajectory_array = self.trajectory_array_generator_deception_3_3slopes(movement)
                    else:
                        points_trajectory_array = self.trajectory_array_generator_deception_3_5slopes(movement)
        return points_trajectory_array

    def trajectory_array_generator_deception_1(self, final_point_coordinates, actual_straight_steps):

        if self.type_deception == 1:
            bool_movement_x = 0
            bool_movement_y = 1
        else:
            bool_movement_x = 1
            bool_movement_y = 0

        points_trajectory_array = numpy.zeros((actual_straight_steps + 6, 2))
        for j in range(6):
            points_trajectory_array[i+j][0] = points_trajectory_array[i+j-1][0] + 0.44 * math.sin(-15*(j+1))
                
        points_trajectory_array[0][0] = final_point_coordinates[0] - actual_straight_steps * self.distance_between_two_straight_steps * bool_movement_x
        points_trajectory_array[0][1] = final_point_coordinates[1] - actual_straight_steps * self.distance_between_two_straight_steps * bool_movement_y

        for i in range(1, actual_straight_steps):
            points_trajectory_array[i][0] = points_trajectory_array[i-1][0] + self.distance_between_two_straight_steps * bool_movement_x
            points_trajectory_array[i][1] = points_trajectory_array[i-1][1] + self.distance_between_two_straight_steps * bool_movement_y
        
        if (final_point_coordinates[0] - self.real_target_coordinates[0]) < 0 or (final_point_coordinates[1] - self.real_target_coordinates[1]) < 0 :
            for j in range(6):
                points_trajectory_array[i+j][0] = points_trajectory_array[i+j-1][0] + 0.44 * math.sin(15*(j+1))
                points_trajectory_array[i+j][1] = points_trajectory_array[i+j-1][1] + 0.44 * math.cos(15*(j+1))
        else:
            for j in range(6):
                points_trajectory_array[i+j][0] = points_trajectory_array[i+j-1][0] + 0.44 * math.sin(-15*(j+1))
                points_trajectory_array[i+j][1] = points_trajectory_array[i+j-1][1] + 0.44 * math.cos(-15*(j+1))
        
        return points_trajectory_array
  
    def trajectory_array_generator_deception_3_1slope(self, movement):
        if movement == 1:
            first_angle = -45
            second_angle = 30
            third_angle = 15
        else:
            first_angle = 45
            second_angle = -30
            third_angle = -15

        step_distance = 0.44
        step_corner_distance = 0.3

        points_trajectory_array = numpy.zeros((9,2))

        points_trajectory_array[0][0] = 0.44 * math.sin(first_angle)
        points_trajectory_array[0][1] =  0.44 * math.cos(first_angle)
        
        for i in range(1,3):
            points_trajectory_array[i][0] = points_trajectory_array[i-1][0] + step_distance * math.sin(first_angle)
            points_trajectory_array[i][1] = points_trajectory_array[i-1][1] + step_distance * math.cos(first_angle)

        for i in range(3):
            points_trajectory_array[i+2][0] = points_trajectory_array[i+2-1][0] + step_corner_distance * math.sin(second_angle*(i+1))
            points_trajectory_array[i+2][1] = points_trajectory_array[i+2-1][1] + step_corner_distance * math.cos(second_angle*(i+1))

        for i in range(3):
            points_trajectory_array[i+5][0] = points_trajectory_array[i+5-1][0] + step_distance * math.sin(third_angle)
            points_trajectory_array[i+5][1] = points_trajectory_array[i+5-1][1] + step_distance * math.cos(third_angle)

        return points_trajectory_array

    def trajectory_array_generator_deception_3_3slopes(self, movement):
        points_trajectory_array =self.trajectory_array_generator_deception_3_1slope(movement)

        points_trajectory_array[9][0] = points_trajectory_array[8][0]
        points_trajectory_array[9][1] = points_trajectory_array[8][1]

        if movement==1:
            points_trajectory_array = numpy.append(points_trajectory_array, self.trajectory_array_generator_deception_3_1slope(2), axis = 0)
        else:
            points_trajectory_array = numpy.append(points_trajectory_array, self.trajectory_array_generator_deception_3_1slope(1), axis = 0)
        return points_trajectory_array
            
     def trajectory_array_generator_deception_3_5slopes(self, movement):

        points_trajectory_array = self.trajectory_array_generator_deception_3_3slopes(movement)
        points_trajectory_array = numpy.append(points_trajectory_array, self.trajectory_array_generator_deception_3_1slope(movement), axis = 0)
        return points_trajectory_array
        

    def get_parameters_line(self):
        m = (self.fake_target_coordinates[1] - self.real_target_coordinates[1]) / (self.fake_target_coordinates[0] - self.real_target_coordinates[0])
        q = - self.real_target_coordinates[0] * m + self.fake_target_coordinates[1]
        return [m,q]

    def get_segment(self, m, q):
        x = q / (m + 1 / m)
        y = - x / m
        return [x, y]

    def distance(self, point1, point2):
        dist = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return dist

    def check_type_deception(self):
        if self.type_deception == 1:
            if 1.4 >= self.set_robot_coordinates[0] >= 1.9: # Go straigth in x
                self.type_deception = 1
            else:
                if 1.4 >= self.set_robot_coordinates[1] >= 1.9: # Go straigth in y
                    self.type_deception = 2
                else:
                    self.type_deception = 3
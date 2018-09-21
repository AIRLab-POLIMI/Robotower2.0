#!/usr/bin/env python
import numpy
import math
import random

import rospy


def get_fake_target(outcome_matrix_robot, outcome_matrix_player, real_target):
    # This function finds the fake target checking the outcome matrix and taking the 2 max values from the robot's outcome
    # matrix. The fake target will be chosen checking the player's outcome matrix. It will be chosen the one that maximizes
    # the fake payoff of the player
    max_r = 0
    index = 0

    for i in range(4):
        if outcome_matrix_robot[i][real_target] > max_r:
            index = i
            max_r =outcome_matrix_robot[i][real_target]

    min_r = numpy.min(outcome_matrix_robot[:, real_target])

    # index_other_max = 0

    # for i in range(4):
    #     if (not outcome_matrix_robot[i][real_target] == max_r) and (outcome_matrix_robot[i][real_target] > min_r):
    #         min_r = outcome_matrix_robot[i][real_target]
    #         index_other_max = i

    # if outcome_matrix_player[real_target][index] > outcome_matrix_player[real_target][index_other_max]:
    #     fake_target = index
    # else:
    #     fake_target = index_other_max
    
    # if not (real_target < 3 and fake_target == real_target + 1) or (real_target == 3 and fake_target == 0):
    #     if fake_target == index:
    #         fake_target = index_other_max
    #     else:
    #         fake_target = index
    fake_target = index

    return fake_target


def get_type_deception(fake_target, real_target, robot_xy, player_xy):

    robot_player_angle = math.atan2(player_xy[1] - robot_xy[1], player_xy[0] - robot_xy[0])

    # TODO check the angle!!!!!

    if (real_target < 3 and fake_target == real_target + 1) or (real_target == 3 and fake_target == 0):
        return 3
        # if (real_target == 0 and 3 / 2 * math.pi > robot_player_angle > 2 * math.pi) or (
        #             real_target == 1 and math.pi > robot_player_angle > 3 / 2 * math.pi) or (
        #             real_target == 2 and math.pi / 2 > robot_player_angle > 1 / 2 * math.pi) or (
        #             real_target == 4 and 0 > robot_player_angle > 1 / 2 * math.pi):
        #     return 1
        # else:
        #     return 3
    else:
        return 4
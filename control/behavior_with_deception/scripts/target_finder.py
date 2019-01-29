#!/usr/bin/env python
import numpy
import math
import rospy


def distance(point1, point2):
    dist = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    return dist


def check_for_negative_values(target_preferences):
    negative_values = []
    for i in range(4):
        if target_preferences[i] < 0:
            negative_values.append(target_preferences[i])

    if len(negative_values) > 0:
        max_neg = numpy.min(negative_values)
        for i in range(4):
            if not target_preferences[i] == 0:
                target_preferences[i] = abs(target_preferences[i] - max_neg) + 1

    return target_preferences


def target_preferences_robot(player_xy, robot_xy, towers_xy, previous_target):

    target_preferences = ([0.0, 0.0, 0.0, 0.0])

    for i in range(4):
        target_preferences[i] = previous_target[i] * (distance(player_xy, towers_xy[i, :]) - distance(towers_xy[i, :], robot_xy))

    target_preferences = check_for_negative_values(target_preferences)
    # target_preferences = normalization_target_arrays(target_preferences)
    return target_preferences


def find_target(target_preferences):
    index_max = 0
    max_t = target_preferences[0]
    for i in range(4):
        if target_preferences[i] > max_t:
            max_t = target_preferences[i]
            index_max = i

    return index_max


def target_preferences_player(player_xy, robot_xy, towers_xy, previous_target):

    target_preferences = ([0.0, 0.0, 0.0, 0.0])

    for i in range(4):
        target_preferences[i] = previous_target[i] * ((1 / distance(towers_xy[i, :], robot_xy)) + (1 / distance(towers_xy[i, :], player_xy)))
    # target_preferences = normalization_target_arrays(target_preferences)
    return target_preferences


def normalization_target_arrays(target_array):

    target_normalized = [x * 50 / numpy.sum(target_array) for x in target_array]

    return target_normalized
#!/usr/bin/env python
import numpy


def correspondence(outcome_matrix_robot, outcome_matrix_player):

    beta = 0

    avg_robot_actions = numpy.mean(outcome_matrix_robot, axis=0)
    avg_player_actions = numpy.mean(outcome_matrix_player, axis=1)

    # for index in range(4):
    #     beta = beta + outcome_variation(outcome_matrix_robot[index][:], outcome_matrix_player[index][:], avg_robot_actions[index], numpy.sum(avg_robot_actions))

    # for index in range(4):
    #     beta = beta + outcome_variation(outcome_matrix_robot[:][index], outcome_matrix_player[:][index], avg_player_actions[index], numpy.sum(avg_player_actions))

    for index in range(4):
        beta = beta + outcome_variation(outcome_matrix_robot[:][index], outcome_matrix_player[:][index], avg_robot_actions[index], numpy.sum(avg_robot_actions))

    for index in range(4):
        beta = beta + outcome_variation(outcome_matrix_robot[index][:], outcome_matrix_player[index][:], avg_player_actions[index], numpy.sum(avg_player_actions))

    return beta/2


def outcome_variation(outcome_values_robot, outcome_values_player, weight, normalization):
    index_max_r = numpy.argmax(outcome_values_robot)
    index_max_p = numpy.argmax(outcome_values_player)

    diff_r = outcome_values_robot[index_max_r] - outcome_values_robot[index_max_p]
    diff_p = outcome_values_player[index_max_r] - outcome_values_player[index_max_p]

    sum_r = abs(outcome_values_robot[index_max_r]) + abs(outcome_values_robot[index_max_p])
    sum_p = abs(outcome_values_player[index_max_r]) + abs(outcome_values_player[index_max_p])

    if sum_r == 0:
        sum_r = 1
    if sum_p == 0:
        sum_p = 1

    result = (weight/normalization) * (diff_r/sum_r) * (diff_p/sum_p)

    return result

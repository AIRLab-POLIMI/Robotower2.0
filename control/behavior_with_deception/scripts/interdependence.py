#!/usr/bin/env python
import numpy

#
# def interdependence(outcome_matrix_robot, outcome_matrix_player):
#
#     avg_robot_actions = numpy.mean(outcome_matrix_robot, axis=0)
#     avg_player_actions = numpy.mean(outcome_matrix_player, axis=1)
#
#     index_max_robot = numpy.argmax(avg_robot_actions)
#     index_max_player = numpy.argmax(avg_player_actions)
#
#     element12 = numpy.delete(outcome_matrix_robot[index_max_player][:], index_max_robot)
#     element21 = numpy.delete(outcome_matrix_robot[:][index_max_robot], index_max_player)
#     element22 = numpy.delete(numpy.delete(outcome_matrix_robot, index_max_player, axis=1), index_max_robot, axis=0)
#
#     interd_outcome_robot = [[outcome_matrix_robot[index_max_player][index_max_robot] , numpy.mean(element12)],
#                           [numpy.mean(element21), numpy.mean(element22)]]
#
#     alpha_r = numpy.sum(numpy.absolute(numpy.absolute(interd_outcome_robot[0][:]) - numpy.absolute(interd_outcome_robot[1][:]))) / numpy.sum(interd_outcome_robot)
#     return alpha_r


def interdependence(outcome_matrix_robot, target_array_robot):
    alpha_r = 0
    for i in range(4):
        max_r = numpy.max(outcome_matrix_robot[:, i])
        min_r = numpy.min(outcome_matrix_robot[:, i])
        diff_outcomes = abs(max_r - min_r)
        sum_outcomes = max_r + min_r
        alpha_r += target_array_robot[i]/numpy.sum(target_array_robot) * diff_outcomes/sum_outcomes
    return alpha_r
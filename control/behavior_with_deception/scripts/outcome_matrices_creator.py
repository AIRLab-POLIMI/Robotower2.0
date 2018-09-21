#!/usr/bin/env python
import numpy
import target_finder


def outcome_matrix_robot(target_array, towers_xy, player_xy, robot_xy):

    sum_distances_player_towers = target_finder.distance(towers_xy[0, :],
                                                         player_xy) + target_finder.distance(
        towers_xy[1, :], player_xy) + target_finder.distance(towers_xy[2, :],
                                                             player_xy) + target_finder.distance(
        towers_xy[3, :], player_xy)

    outcome_matrix = numpy.zeros((4, 4))

    for column in range(4):
        for row in range(4):
            if row == column:
                outcome_matrix[row][column] =  target_finder.distance(towers_xy[row, :], robot_xy) / 1 #target_array[column] *
            else:
                outcome_matrix[row][column] = 100 * (
                    target_array[column] / numpy.sum(target_array) * target_finder.distance(player_xy,
                                                                                                towers_xy[row,
                                                                                                :]))/sum_distances_player_towers
    return outcome_matrix


def outcome_matrix_player(target_array, towers_xy, robot_xy):

    sum_distances_robot_towers = 0
    outcome_matrix = numpy.zeros((4, 4))

    for i in range(4):
        sum_distances_robot_towers += target_finder.distance(towers_xy[i, :], robot_xy)

    for row in range(4):
        for column in range(4):
            if row == column:
                outcome_matrix[row][column] = 100 * 1 / target_finder.distance(towers_xy[column, :], robot_xy)
            else:
                outcome_matrix[row][column] = 1 * (target_array[row] / numpy.sum(target_array) * target_finder.distance(towers_xy[column, :], robot_xy) / sum_distances_robot_towers)

    return outcome_matrix

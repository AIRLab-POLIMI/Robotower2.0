import target_finder


def player_model_exploration(player_xy, tower_xy, robot_xy, cont):

    player_model_exploration.distance_robot_player += target_finder.distance(player_xy, robot_xy)

    for tower in range(4):
        if target_finder.distance(tower_xy[tower, :], player_xy) < 1:
            player_model_exploration.check_allowed_distance = 1
            player_model_exploration.target_tower = tower
            player_model_exploration.saved_distance = target_finder.distance(player_xy, robot_xy)

    if player_model_exploration.check_allowed_distance == 1:
        if target_finder.distance(player_xy, robot_xy) <= player_model_exploration.saved_distance * 0.5:
            for tower in range(4):
                if target_finder.distance(tower_xy[tower, :], robot_xy) <= 2.0:
                    player_model_exploration.allowed_distance = target_finder.distance(tower_xy[tower, :], robot_xy)
            player_model_exploration.check_allowed_distance = 0

        player_model_exploration.saved_distance = target_finder.distance(player_xy, robot_xy)

    return [player_model_exploration.distance_robot_player/cont, player_model_exploration.allowed_distance]


player_model_exploration.distance_robot_player = 0
player_model_exploration.check_allowed_distance = 0
player_model_exploration.target_tower = 0
player_model_exploration.saved_distance = 0
player_model_exploration.allowed_distance = 0


def check_model_player(allowed_distance, average_robot_player_distance):
    if (allowed_distance>=0.75 and average_robot_player_distance<=0.25):
        return 1
    else:
        if (allowed_distance<=0.25 and average_robot_player_distance>=0.75):
            return 2
        else:
            return 3


# def reaction_detection(player_xy, robot_xy):
#     if target_finder.distance(player_xy, robot_xy) <= 0.9:
#         return False
#     else:
#         return True


# def player_model_definition(distance_robot_player, distance_allowed_robot_tower, reaction):


# def player_model_on_line(robot_player_xy, robot_tower_xy):
#     player_model_on_line.cont += 1
#
# player_model_on_line.cont = 0
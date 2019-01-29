float64[] player_tower_distances    # distances between player and towers
float64[] robot_tower_distances     # distances between robot and towers
float64 player_robot_distance       # distance between player and robot

float64[] player_tower_distances_variation  # variation in distance from previous observation
float64[] robot_tower_distances_variation   # variation in distance from previous observation 
float64 player_robot_distance_variation     # variation in distance from previous observation

geometry_msgs/Point player_position_wrt_map   # position of player in the map frame
geometry_msgs/Point robot_position_wrt_map    # position of robot in the map frame
float64 robot_rotation_wrt_map                # robot frame rotation wrt map frame

float64[] velocity_projections  # projection of robot velocity on segements connecting it to towers


bool is_stationary  # flag to indicate if the player is stationary
int8 target_tower   # index of the tower the player is moving to 
int8 closest_tower_index # closest tower to the player
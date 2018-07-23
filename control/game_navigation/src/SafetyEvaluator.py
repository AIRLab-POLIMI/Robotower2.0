import numpy as np


class SafetyEvaluator:
    ''' Helper class to implement different kind of safety checks in different situations '''
    # ALLOWED MODES OF EVALUATION
    self.SEARCHING_TOWER = 0   # WE FOUND SOMETHING IN PROXIMITY AND WE'RE CLOSE TO THE GOAL
    self.APPROACHING_TOWER = 1  # WE FOUND SOMETHING IN DANGER_ZONE (i.e < DONTCARE ) AND WE'RE CLOSE TO THE GOAL
    self.DEFAULT = 2            # WE'RE AWAY FROM THE GOAL
    self.SCAN_LENGHT = 1000
    
    self.TOWER_RADIUS = 0.07    # TODO: CHECK THIS ESTIMATED RADIUS OF TOWER
    # TODO: UPDATE THIS PARAMETER WITH PHYSICAL EXPERIMENTS
    self.DANGER_ZONE = 75       # WIDTH OF DANGER ZONE, EXPRESSED IN INDEXES: IF WE FIND SOMETHING IN 75 INDEXES ( = 27 deg) IN EITHER DIRECTION FROM TOWER, PLAYER HAS BLOCKED US


    def __init__(self):
        # Assuming that we start from a safe condition and away from towers
        self.mode = self.DEFAULT
        self.filtered_scan = []
        self.closest_point_index = 0
        # Set of indexes corresponding as the tower
        # Initialize to complete scan
        self.estimated_tower_indexes = range(0, self.SCAN_LENGHT)
        
        # Set of indexes we consider as obstructing the tower
        self.danger_zone_indexes = []
        
        

    def set_mode(self, new_mode):
        if(self.mode == self.AVOIDING):
            # We can exit the AVOIDING mode only if we're going to DEFAULT -> all obstacles are clear
            if(new_mode == self.DEFAULT):
                self.mode = mode
                return
        self.mode = new_mode

    def estimate_index_window(self):
        # Returns the indexes of the estimated chunck occupied by the tower
        min_dist = self.filtered_scan[self.closest_point_index]

        # angle between the robot and one of the side of the projected radius on the tangent to closest point
        # NOTE: see picture for better understanding
        angular_semi_window = np.arctan2(self.TOWER_RADIUS, min_dist)

        # Convert the angle in number of indexes
        index_semi_window = int( angular_semi_window / (360.0 / self.SCAN_LENGHT) )

        # Return the range of such lenght centered in the closest point
        # TODO Handle scan edges
        self.estimated_tower_indexes = range(closest_point_index - index_semi_window, closest_point_index + index_semi_window)

        # TESTING check that these scan indexes actually contains the tower. Should containd values padded by some 0s at the beginnin and end
        # rospy.loginfo("Estimated window occupied by tower: {}".format(self.scan_filtered[estimated_tower_indexes[0]:estimated_tower_indexes[-1]]))

    def estimate_danger_indexes(self):
        # Return the indexes where we consider a player has blocked the robot if we find something
        # TODO Handle scan edges
        danger_right = range(self.estimated_tower_indexes[0] - self.DANGER_ZONE, 
            self.estimated_tower_indexes[0] - 1)
        danger_left = range( self.estimated_tower_indexes[-1] + 1, 
            self.estimated_tower_indexes[-1] + self.DANGER_ZONE)

        self.danger_zone_indexes = danger_right.extend(danger_left)



    def check_consistency(self, force_update=False):
        # Checks consistency between subsequent scans
        
        # We check the chunk that we overestimated the cycle before
        starting_index = self.estimated_tower_indexes[0]
        final_index = self.estimated_tower_indexes[-1]
        # Extract the chunk
        tower_chunk = self.filtered_scan[starting_index:final_index]
        masked_tower_chunk = np.ma.masked_equal(tower_chunk, 0) # Mask indexes not representing a danger
        min_index = masked_tower_chunk.argmin()
        

        min_index_displacement = abs(self.closest_point_index - min_index)
        # rospy.loginfo("The index of the closest point of the tower is {}".format(min_index))

        if ( min_index_displacement < 10 ): #TODO Adjust this parameter (10 allows an oscillation of 3.6 * 2 degrees)
            # We check the assumption that the closest point is almost stationary and update it
            self.closest_point_index = min_index
        else:
            if(force_update):
                # We're in the zone between proximity and danger, it's a too early stage to make a call
                self.closest_point_index = min_index
            else:
            # Something happened: maybe the player has jumped in front of the robot???
            # TODO: Check it with physical experiments
                pass


    def reset(self):
        ''' Sets the state of the evaluator back to the initial one '''
        self.estimated_tower_indexes = range(0, self.SCAN_LENGHT)
        self.closest_point_index = 0
        self.filtered_scan = []
        self.danger_zone_indexes = []
        self.mode = self.AVOIDING


    def evaluate_safety(self, filtered_scan):
    ''' Given the current filtered scan and the current mode, 
    returns wether the situation is safe (no obstacle to reacr) or not 
    @filtered_scan: array containing the distance of an obstacle if it's below dontcare treshold '''
    self.filtered_scan = filtered_scan

    if(self.mode == self.SEARCHING_TOWER):
        # We're not yet in a danger situation, we start identifing which indexes are the tower
        self.check_consistency(force_update=True)
        self.estimate_index_window()
        return True # No danger

    else if(self.mode == self.APPROACHING_TOWER):
        # We found something below DONTCARE
        self.check_consistency()
        self.estimate_index_window()
        self.estimate_danger_indexes()

        for danger_index in self.danger_zone_indexes:
            if( filtered_scan[danger_index] != 0):
                # There's something in an angular position around the tower
                # We interpret this evidence as a player putting himself between robot and tower
                reset() # Will interpret everything as an obstacle while we move away
                return False
        # None of the indexes we consider obstructing the tower is being occupied
        return True

    else if(self.mode = self.DEFAULT):
        # We're not approaching any tower, consider everything an obstacle
        if (sum(filtered_scan) == 0):
            return True
        else:
            return False
    
    else if (self.mode == self.AVOIDING):
        return False



###### AUXILIARY FUNCTIONS TO BE PLUGGED IN NAVIGATION #########

def filter(self, scan):
    filtered_values = []
    self.danger = False
    self.search = False
    for value in self.current_scan.ranges:
        if value < 0.8:
            self.search = True
            if value < self.DONTCARE:
                self.danger = True
            filtered_values.append(value)
        else:
            filtered_values.append(0)
    
    return filtered_values

def set_evaluator_mode(self):
    if(self.is_near_goal()):
        if(self.danger):
            self.safety_evaluator.set_mode(self.APPROACHING_TOWER)
        else if(self.search): # Needed to avoid to pass an empty filtered scan 
            self.safety_evaluator.set_mode(self.SEARCHING_TOWER)
    else:
        self.safety_evaluator.set_mode(self.DEFAULT)
            
def is_near_goal(self):
    # define initial e final point when the robot receive the id of the targeted tower
    xd = (self.robot_estimated_pose[0][0], self.TOWERS[self.current_goal][0])
    yd = (self.robot_estimated_pose[1][0], self.TOWERS[self.current_goal][1])

    # define the robot deviation from the required trajectory
    delta_x = xd[1] - xd[0]
    delta_y = yd[1] - yd[0]

    goal_distance = (delta_x**2 + delta_y**2)**0.5

    if goal_distance < 0.8:
        return True
    return False

###########################################################################################




####### TO EVALUATE SAFETY CONDITION IN LaserScanManager #######

filtered_scan = self.filter(self.current_scan.ranges)
self.set_evaluator_mode()
self.is_safe = self.safety_evaluator.evaluate_safety(filtered_scan)
 
################################################################
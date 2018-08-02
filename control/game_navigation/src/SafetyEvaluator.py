import numpy as np
import rospy

class SafetyEvaluator:
    ''' Helper class to implement different kind of safety checks in different situations '''
    # ALLOWED MODES OF EVALUATION
    SEARCHING_TOWER = 0    # WE FOUND SOMETHING IN PROXIMITY AND WE'RE CLOSE TO THE GOAL
    APPROACHING_TOWER = 1  # WE FOUND SOMETHING IN DANGER_ZONE (i.e < DONTCARE ) AND WE'RE CLOSE TO THE GOAL
    DEFAULT = 2            # WE'RE AWAY FROM THE GOAL
    AVOIDING = 3           # WE HAVE BEEN BLOCKED BY THE PLAYER
    SCAN_LENGHT = 1000
    
    TOWER_RADIUS = 0.06    # MEASURED RADIUS OF TOWER: 6cm
    # TODO: UPDATE THIS PARAMETER WITH PHYSICAL EXPERIMENTS
    DANGER_ZONE = 75       # WIDTH OF DANGER ZONE, EXPRESSED IN INDEXES: IF WE FIND SOMETHING IN 
                                # 75 INDEXES ( = 27 deg) IN EITHER DIRECTION FROM TOWER, PLAYER HAS BLOCKED US


    def __init__(self, dontcare_thd):
        # Assuming that we start from a safe condition and away from towers
        self.mode = self.DEFAULT
        self.DONTCARE = dontcare_thd
        self.FILTER_TRESHOLD = 0.8
        self.MIN_FILTERING_TRESHOLD = 0.35
        self.filtered_scan = []
        self.closest_point_index = 0
        # Set of indexes corresponding as the tower
        # Initialize to complete scan
        self.tower_first_index = 0
        self.tower_last_index = self.SCAN_LENGHT - 1
        
        # Set of indexes we consider as obstructing the tower
        self.danger_zone_indexes = []
        self.unsafe_counter = 0
        

    def set_mode(self, new_mode):
        if(self.mode == SafetyEvaluator.AVOIDING):
            print "TRYING TO CHANGE AVOIDING MODE"
            # We can exit the AVOIDING mode only if we're going to DEFAULT -> all obstacles are clear
            if(new_mode == SafetyEvaluator.DEFAULT):
                self.mode = new_mode
            # Otherwise return without changing
            return
        self.mode = new_mode

    def estimate_index_window(self):
        ''' Estimates the length of the window that will be occupied by the tower.
            To do so we take the smallest scan value inside the PREVIOUS window (i.e. during the last cycle). We assume that the tower won't move,
            meaning it will occupy more or less the same window. Given this value, we can approximate the window lenght by trigonometric calculations,
            knowing the tower radius'''
        
        # Returns the indexes of the estimated chunck occupied by the tower
        min_dist = self.filtered_scan[self.closest_point_index]
        # rospy.logerr("Minimum distance at index {} detected {}".format(self.closest_point_index, min_dist))

        # angle between the robot and one of the side of the projected radius on the tangent to closest point
        # NOTE: see picture for better understanding
        angular_semi_window = np.arctan2(SafetyEvaluator.TOWER_RADIUS, min_dist)
        angular_semi_window_deg = np.rad2deg(angular_semi_window)

        # Convert the angle in number of indexes
        # Need to do some conversion from radians to degrees
        index_semi_window = int( angular_semi_window_deg / (360.0 / SafetyEvaluator.SCAN_LENGHT) )
        # rospy.logerr("Window semi lenght: {}".format(index_semi_window))

        # Return the range of such lenght centered in the closest point
        # TODO Handle scan edges
        self.tower_first_index = self.closest_point_index - index_semi_window
        rospy.logwarn("Starting index: {}".format(self.tower_first_index))
        if(self.tower_first_index < 0):
            error_start = abs(self.tower_first_index)
            self.tower_first_index = self.SCAN_LENGHT - error_start

        # rospy.logwarn("Starting index corrected: {}".format(self.tower_first_index))

        self.tower_last_index = self.closest_point_index + index_semi_window
        # rospy.logwarn("Final index : {}".format(self.tower_last_index))
        if(self.tower_last_index >= self.SCAN_LENGHT):
            error_final = self.tower_last_index - self.SCAN_LENGHT
            self.tower_last_index = error_final

        # rospy.logwarn("Final index corrected: {}".format(self.tower_last_index))

    def adjust_tower_window(self):
        """ We can approximate with high confidence the lenght of the window where the tower is going to be, 
            but we have trouble centering it. Usually we get most of the tower inside our window, but since it's not centered we
            may miss the leftmost or rightmost pieces of it: they will be left outside our window. We need to rotate the window in order
            to center the tower and correct our estimate.
            To do so we exploit the fact that if the window is too rotated to the left, the leftmost scans are going to be empty:
            we then need to rotate the window to the right until the leftmost scan contains some evidence of tower.
            Then we do the same for windows that are too rotated to right, rotating them to the left.
            After these two rotations the tower will be in our window, so we can center it
        """

        self.extend_window()
        starting_index = self.tower_first_index
        final_index = self.tower_last_index

        clogged_right = False # True if we can't rotate in this direction because of obstacle
        clogged_left = False  # True if we can't rotate in this direction because of obstacle

        # rospy.loginfo("Tower chunk before rotation {}".format(self.extract_tower_chunk()))

        handle_edge = False
        if(self.tower_first_index > self.tower_last_index): 
            # The tower is behind us, rotate in the opposite direction
            print "HANDLING EDGE CASE"
            handle_edge = True       

        ### ROTATING RIGHT AS MUCH AS POSSIBLE ###
        performed_rotations = 0
        while(self.filtered_scan[self.tower_last_index] == 0 and self.tower_last_index != starting_index): # While we have clean scan on the left
            # if(handle_edge):
            #     self.rotate_left()
            # else:
            self.rotate_right()
            #final_index -= 1
            performed_rotations += 1 # DEBUG Counter of rotations performed
        if(performed_rotations == 0):
            # If we didn't rotate -> WE HAVE NO ROOM EITHER LEFT OR RIGHT -> PROBLEM?
            clogged_right = True
            # rospy.logwarn("Couldn't rotate right by any position...")
        
        # rospy.loginfo("Tower chunk after right rotation {}".format(self.extract_tower_chunk()))
        starting_index = self.tower_first_index
        final_index = self.tower_last_index

        ### ROTATING LEFT AS MUCH AS POSSIBLE ###
        performed_rotations = 0
        while(self.filtered_scan[self.tower_first_index] == 0 and self.tower_first_index != final_index): # While we have clean scan on the right
            # if(handle_edge):
            #     self.rotate_right()
            # else:
            self.rotate_left()
            #starting_index += 1
            performed_rotations += 1 # DEBUG Counter of rotations performed
        if(performed_rotations == 0):
            # If we didn't rotate -> WE HAVE NO ROOM EITHER LEFT OR RIGHT -> PROBLEM?
            clogged_left = True
            rospy.logwarn("Couldn't rotate left by any position...")

        if(clogged_right and clogged_left):
            # If we didn't rotate -> WE HAVE NO ROOM EITHER LEFT OR RIGHT -> PROBLEM?
            rospy.logwarn("Couldn't rotate neither left or right by any position...")
            return False
        # rospy.loginfo("Tower chunk after left rotation {}".format(self.extract_tower_chunk()))
        self.center_window()
        # rospy.loginfo("Tower chunk after centering rotation {}".format(self.extract_tower_chunk()))
        # self.print_tower_contour()

        return True

    def rotate_right(self):
        # Rotate the field of view where the tower is supposed to be from left to right one position, mantaining its lenght
        self.tower_first_index -= 1
        self.tower_last_index -= 1

        if(self.tower_first_index == -1):
            # We went to the other side of the scan
            self.tower_first_index = self.SCAN_LENGHT - 1

    def rotate_left(self):
        # Rotate the field of view where the tower is supposed to be from right to left one position, mantaining its lenght
        self.tower_first_index += 1
        self.tower_last_index += 1

        if(self.tower_last_index == self.SCAN_LENGHT):
            self.tower_last_index = 0

    def center_window(self):

        padding_right = 0
        padding_left = 0

        index = self.tower_first_index
        while(self.filtered_scan[index] == 0): # While I find empty scans on the right
            padding_right += 1
            index += 1
            if (index) == self.SCAN_LENGHT:
                # Start over if we reached scan edge
                index = 0

        index = self.tower_last_index
        while(self.filtered_scan[index] == 0): # While I find empty scans on the left
            padding_left += 1
            index -= 1
            if(index == 0):
                # Start over if we reached scan edge
                index = self.SCAN_LENGHT - 1 

            
        # rospy.logwarn("Padding left: {}     Padding right:{}".format(padding_left, padding_right))
        total_padding = padding_right + padding_left
        semi_padding = total_padding / 2    # Integer division is ok, no need for high precision

        if(padding_left > padding_right):
            # We have more room on the left
            # Rotate until we reduce the left padding to half of the total padding
            while(padding_left > semi_padding):
                self.rotate_right()
                padding_left -= 1
        else:
            # We have more room on the rigth
            # Rotate until we reduce the right padding to half of the total padding
            while(padding_right > semi_padding):
                self.rotate_left()
                padding_right -= 1

    def extend_window(self, extension_tolerance=10):
        self.tower_first_index -= extension_tolerance
        self.tower_last_index +=  extension_tolerance

    
    def check_tower_estimation_window(self):
        if(self.filtered_scan[self.tower_first_index] != 0 or self.filtered_scan[self.tower_last_index] != 0):
            return False
        return True
        

    def estimate_danger_indexes(self):
        # Return the indexes where we consider a player has blocked the robot if we find something
        # TODO Handle scan edges
        
        danger_right = range(self.tower_first_index - SafetyEvaluator.DANGER_ZONE, 
            self.tower_first_index - 1)
        
        danger_left = range( self.tower_last_index + 1, 
            self.tower_last_index + SafetyEvaluator.DANGER_ZONE)

        self.danger_zone_indexes = danger_right + danger_left

    def extract_tower_chunk(self):
         # Extract the chunk 
        if(self.tower_first_index > self.tower_last_index):
            # The tower is on the edge of the scan
            print "EDGE!!!"
            tower_chunk = self.filtered_scan[self.tower_first_index: self.SCAN_LENGHT - 1] + self.filtered_scan[0 : self.tower_last_index]
        else:
            tower_chunk = self.filtered_scan[self.tower_first_index: self.tower_last_index]

        return tower_chunk



    def check_consistency(self, force_update=False):
        # Checks consistency between subsequent scans
        
        # We check the chunk that we overestimated the cycle before
        starting_index = self.tower_first_index
        final_index = self.tower_last_index

        tower_chunk = self.extract_tower_chunk()
        
        masked_tower_chunk = np.ma.masked_equal(tower_chunk, 0.0) # Mask indexes not representing a danger
        min_index = masked_tower_chunk.argmin() + starting_index    # Calculates closest point in tower chunk and calculates the corresponding index wrt filtered_scan
        if(min_index >= self.SCAN_LENGHT):
            error = min_index - self.SCAN_LENGHT
            min_index = error

        min_index_displacement = abs(self.closest_point_index - min_index)

        # TODO force first update of min_index_tower
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
        ''' Sets the state of the evaluator back to the initial one. To be called when we abort going to tower while we where APPROACHING '''
        self.tower_first_index = 0
        self.tower_last_index = self.SCAN_LENGHT - 1
        self.FILTER_TRESHOLD = 0.8
        self.closest_point_index = 0
        self.filtered_scan = []
        self.danger_zone_indexes = []
        self.mode = SafetyEvaluator.AVOIDING


    def evaluate_safety(self, filtered_scan):
        ''' Given the current filtered scan and the current mode, 
        returns wether the situation is safe (no obstacle to react) or not 
        @filtered_scan: array containing the distance of an obstacle if it's below dontcare treshold '''
        self.filtered_scan = filtered_scan  # If we have room between tower and walls

        if(self.mode == SafetyEvaluator.SEARCHING_TOWER):
            # We're not yet in a danger situation, we start identifing which indexes are the tower
            self.check_consistency(force_update=True)
            self.estimate_index_window()
            return True # No danger TODO think if should do default check or we assume the player can't interpose between searching and approaching

        elif(self.mode == SafetyEvaluator.APPROACHING_TOWER):
            # We found something below DONTCARE
            self.check_consistency(force_update=True)
            self.estimate_index_window()
            adjust_successful = self.adjust_tower_window()
            if(not self.check_tower_estimation_window()):
                rospy.logerr("THE PLAYER IS BLOCKING THE TOWERRRR")
                self.print_tower_contour()
                self.reset()
                return False
            self.estimate_danger_indexes()
            for danger_index in self.danger_zone_indexes:
                if( self.filtered_scan[danger_index] != 0):
                    if(not self.check_contour(danger_index)):
                        rospy.logerr("Found something at index: {} with distance: {}".format(danger_index, filtered_scan[danger_index]))
                        self.print_tower_contour()
                        self.print_danger_scan()
                        self.reset()
                        return False
            return True

        elif(self.mode == SafetyEvaluator.DEFAULT):
            # We're not approaching any tower, consider everything an obstacle
            return self.default_check()

        elif (self.mode == SafetyEvaluator.AVOIDING):
            # The player has blocked us, rely on Davide's obstacle avoidance until we're safe again
            return False

    def default_check(self):
        for value in self.filtered_scan:
            if value < self.DONTCARE and value > 0:
                return False
        return True

    def check_contour(self, danger_index):
        if(self.filtered_scan[danger_index - 1] != 0 and self.filtered_scan[danger_index + 1] != 0):
            # If both the indexes around the single point obstacle contain obstacles
            # We are not safe
            return False
        return True

    def print_tower_contour(self):
        window = 20
        rospy.loginfo("The unextended tower indexes are from {} to {}".format(self.tower_first_index, self.tower_last_index))
        rospy.loginfo("The unextended tower chunck is {}".format(self.extract_tower_chunk))
        rospy.loginfo("Danger Indexes: {}".format(self.danger_zone_indexes))

    def print_danger_scan(self):
        start_danger = self.danger_zone_indexes[0]
        start_tower = self.tower_first_index - 1

        # rospy.loginfo("Right danger zone: {}".format(self.filtered_scan[start_danger:start_tower]))

        end_tower = self.tower_last_index + 1
        end_danger = self.danger_zone_indexes[-1]

        # rospy.loginfo("Left danger zone: {}".format(self.filtered_scan[end_tower:end_danger]))

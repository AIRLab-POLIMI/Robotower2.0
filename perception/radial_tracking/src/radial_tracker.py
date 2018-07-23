#!/usr/bin/python
import rospy
import numpy as np
import math
import copy
from abc import ABCMeta
from abc import abstractmethod
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32, Float32MultiArray
from player_tracker.msg import PersonArray, Person


"""
Allow an object to alter its behavior when its internal state changes.
The object will appear to change its class.
"""

import abc


class Context:
    """
    Define the interface of interest to clients.
    Maintain an instance of a ConcreteState subclass that defines the
    current state.
    """

    def __init__(self, state, recovery_timeout):
        self._state = state
        self._lost = False
        self.recovery_timeout = recovery_timeout

    def request(self, last_message, listener = None):
        """ Returns the angle where to rotate the robot, according to current tracker state"""
        return self._state.handle(last_message, listener)

    def evaluate_state(self, is_lost):
        next_state = self._state.next_state(is_lost, recovery_timeout)
        self._state = next_state


class State():
    """
    Define an interface for encapsulating the behavior associated with a
    particular state of the Context.
    """
    __metaclass__ = ABCMeta

    @abstractmethod
    def handle(self):
        pass

    @abstractmethod
    def next_state(self, is_lost):
        pass


class TrackingActive(State):
    """
    State associated with current tracking of person
    """

    def handle(self, player_info, listener):
        """ Rotate towards the player """
        #rospy.loginfo("Handling correct tracking")
        if (player_info.position != None):
            player_position = self.transformPoint(listener, player_info.position)
            angle = np.arctan2(player_position[1], player_position[0])
            return angle
        else:
            return 0.0


    def transformPoint(self, listener, ps):
        tweaked_ps = copy.deepcopy(ps)
        tweaked_ps.header.stamp = rospy.Time(0) # Changing time to get point coordinates wrt robot's CURRENT position
        try:
            listener.waitForTransform(
                        "base_link", tweaked_ps.header.frame_id, rospy.get_rostime(), rospy.Duration(1.0))
            tweaked_ps = listener.transformPoint("base_link", tweaked_ps)
        except (tf.Exception):
            return (0,0)
        return (tweaked_ps.point.x, tweaked_ps.point.y)
        
    
    def next_state(self, is_lost, timeout):
        if(is_lost):
            # We've just lost track of the player
            return TrackingLost()
        else:
            return self


class TrackingLost(State):
    """
    State associated with recent loss of person tracking
    """
    def __init__(self):
        self.start_time_ = rospy.get_rostime()
        self.quadrant_index_decoder = dict([(-2, "3rd"), (-1, "4th"), (0, "1st"), (1, "2nd"), (2, "2nd")])
        self.tower_index_decoder = dict([(-2, "tower_4"), (-1, "tower_1"), (0, "tower_2"), (1, "tower_3"), (2, "tower_3")])
        

    def handle(self, last_msg, listener):
        """ Rotate towards the last tower the player intended to reach"""
        #rospy.loginfo("Handling lost tracking")

        index = self.identify_intended_tower(last_msg)
        #rospy.logerr("Heading to quadrant: {}".format(self.quadrant_index_decoder[index]))
        #rospy.logerr("Heading to tower: {}".format(self.tower_index_decoder[index]))

        listener.waitForTransformFull( self.tower_index_decoder[index], rospy.Time(0), "/base_link", rospy.Time(0), "map", timeout=rospy.Duration(2))
        (trans, rot) = listener.lookupTransformFull( self.tower_index_decoder[index] , rospy.Time(0), "/base_link", rospy.Time(0), "map") 
        tf_align_angle = tf.transformations.euler_from_quaternion(rot)

        trans_align_angle = np.arctan2(trans[1], trans[0]) # rotation needed to face player once aligned
        #rospy.loginfo("Angle trans in deg: {}".format(np.rad2deg(trans_align_angle - np.pi)))
        delta = tf_align_angle[2] - (trans_align_angle - np.pi)
        
        
        if(delta > np.pi):
            # rotate by the minimum angle possible
            delta = delta - 2*np.pi

        return -delta #TODO fix

    def identify_intended_tower(self, last_msg):
        """ What tower did the player intended to reach? """
        motion_angle = np.arctan2(last_msg.velocity[1], last_msg.velocity[0])
        # Returns the sector where the player is headed wrt its position
        index = int(motion_angle / (np.pi/2.0))     # Divide the angle by pi/2 -> 90 deg
        if(motion_angle < 0):
            index -= 1
        return index
        
    def check_recovery_mode(self, recovery_thd=2):
        ''' Check if we've lost track of player for too much time '''
        if( (rospy.get_rostime().secs - self.start_time_.secs) < recovery_thd ):
            return False
        return True

    def next_state(self, is_lost, timeout):
        if(is_lost):
            # We've lost track of the player
            if self.check_recovery_mode(recovery_thd=timeout):
                # We've lost track for too much time
                return Recovery()
            else:
                return self
        else:
            # We've recovered track of the player
            return TrackingActive()
        


class Recovery(State):
    """
    State associated with long time unseen player
    """

    def handle(self, last_player_msg, listener):
        """ Rotate towards the center of the field """
        #rospy.loginfo("Handling recovery mode tracking")

        listener.waitForTransformFull( "/playground_center", rospy.Time(0), "/base_link", rospy.Time(0), "map", timeout=rospy.Duration(2))
        (trans, rot) = listener.lookupTransformFull( "/playground_center" , rospy.Time(0), "/base_link", rospy.Time(0), "map") 
        tf_align_angle = tf.transformations.euler_from_quaternion(rot)

        trans_align_angle = np.arctan2(trans[1], trans[0]) # rotation needed to face center of field once aligned
        delta = tf_align_angle[2] - (trans_align_angle - np.pi)
        
        
        if(delta > np.pi):
            # rotate by the minimum angle possible
            delta = delta - 2*np.pi

        return -delta

    def next_state(self, is_lost, timeout):
        if(is_lost):
            return self
        else:
            # We've recovered track of the player
            return TrackingActive()

 
class PlayerInfo(object):
    @property
    def time(self):
        return self._time

    @time.setter
    def time(self, value):
        self._time = value

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, value):
        self._position = value

    @property
    def velocity(self):
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        self._velocity = value

    def __init__(self):
        self.position = None
        self.velocity = (0,0)
        self.time = rospy.get_rostime()



class RadialTracking():

    def __init__(self, lost_timeout, recovery_timeout, is_lost = True):
        # Instantiating the initial state of the tracker
        track_active = TrackingActive()
        # Saving the context of the tracker in a variable
        self.ctx = Context(track_active, recovery_timeout)

        # Instantiating angle publisher
        self.pub_angle = rospy.Publisher('angle', Float32, queue_size=1)

        # Instantiating subscribers to relevant messages
        self.sub_people = rospy.Subscriber('people_tracked', PersonArray, self.callback_person_array)

        self.is_lost = is_lost
        self.last_player_msg = Person()
        self.player_info = PlayerInfo()

        self.robot_frame = "/base_link"
        self.tf_listener = tf.TransformListener()

        self.lost_timeout = lost_timeout

    def callback_person_array(self, msg):
        """callback for PersonArray msgs """
        self.is_lost = False
        max_confidence = float("-inf")
        best_track = None
        # get best person hipothesis
        for person in msg.people:
            if person.confidence > max_confidence:
                max_confidence = person.confidence
                best_track = person
        

        if max_confidence != float("-inf"):
            self.player_info.velocity = (best_track.velocity.x, best_track.velocity.y)
            self.player_info.time = rospy.get_rostime()

            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.point.x = best_track.pose.position.x
            ps.point.y = best_track.pose.position.y

            self.player_info.position = ps            
            
        
    def evaluate_timeout(self):
        ''' Check if the last person received was too old '''
        return ( rospy.get_rostime().to_sec() - self.player_info.time.to_sec() ) > self.lost_timeout
        #return False # TEST TRACK ACTIVE
        #return True  # TEST TRACK LOST

    def run(self):
        r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            self.is_lost = self.evaluate_timeout()
            self.ctx.evaluate_state(self.is_lost)
            rotation_angle = self.ctx.request(self.player_info, self.tf_listener) # Using last know player info
            #rospy.logerr("Rotate by {}".format(np.rad2deg(-rotation_angle)))
            msg = Float32()
            msg.data = rotation_angle
            self.pub_angle.publish(msg)

            r.sleep()
            

if __name__ == '__main__':
    rospy.init_node('radial_tracker')
    lost_timeout = rospy.get_param('lost_timeout')
    recovery_timeout = rospy.get_param('recovery_timeout')
    tracker = RadialTracking(lost_timeout, recovery_timeout)
    tracker.run()

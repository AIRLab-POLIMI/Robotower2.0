#!/usr/bin/python
import rospy
import random
import traceback
import operator
import numpy as np
import tf
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Float32, Float32MultiArray
from player_tracker.msg import PersonArray, Person
from radial_tracking.msg import SectorProbabilities
from matplotlib import pyplot as plt
from matplotlib import animation

__version__ = '0.0.1'
__license__ = 'BSD'
__author__ = ['Ewerton Lopes', 'Stefano Boriero']
__author_email__ = ['ewerton.lopes@polimi.it',
                    'stefano.boriero@mail.polimi.it']


class Particle(object):
    def __init__(self, radius, alpha):
        self.radius = radius
        self.alpha = alpha

    # def update(self, observed_displacement, observed_orientation):
    #     """updates particle position based on its velocity"""
    #     curr_x = self.radius * np.cos(self.alpha)
    #     curr_y = self.radius * np.sin(self.alpha)
    #     new_x = curr_x + (np.cos(observed_orientation) * observed_displacement)
    #     new_y = curr_y + (np.sin(observed_orientation) * observed_displacement)
    #     self.alpha = np.arctan2(new_y,new_x)
    #     self.radius = np.sqrt(new_x**2 + new_y**2)

    def update(self, velocity, dt, mult_factor=1):
        """Updates particle position wrt robot base link using player's velocity"""
        # Get initial position in Cartesian frame
        curr_x = self.radius * np.cos(self.alpha)
        curr_y = self.radius * np.sin(self.alpha)

        # Velocity encodes the direction
        new_x = curr_x + velocity.x * dt * mult_factor
        new_y = curr_y + velocity.y * dt * mult_factor

        rospy.logerr("Difference displacement{}".format(abs(curr_x - new_x)))

        # Update the internal representation of the particle
        self.alpha = np.arctan2(new_y, new_x)
        self.radius = np.sqrt(new_x**2 + new_y**2)

    def get_sector(self, sec_size=0.785398):
        """TODO: Returns the sector"""
        # NOTE adding an offset to get consistent naming -> sector number in [0,7]
        return int(self.alpha / sec_size) + 4


class ParticleFilter(object):

    def __init__(self, n_particles):
        self.N_PARTICLES = n_particles
        self.particles = []

    def initialize(self):
        """Randomly initialize particles"""
        return [Particle(np.random.uniform(1, 2, 1)[0], np.random.uniform(-np.pi, np.pi, 1)[0]) for i in range(self.N_PARTICLES)]

    def propagate(self, observed_displacement):
        """Updates particles"""
        raise NotImplemented("Not implemented!")

    def normalize_weights(self, weights):
        """perform normalization"""
        s = sum(weights)
        return [weights[i]/s for i in range(len(weights))]

    def measurement_prob(self, msg):
        """get measurement probability"""
        raise NotImplemented("Not implemented!")

    def resample(self, weights):
        """Implements the sampling wheel method for keeping the best weighted particle"""

        new_particles = []
        # Getting a random index for sampling
        index = int(random.random() * self.N_PARTICLES)
        beta = 0.0
        # getting maximum weight
        mw = max(weights)
        # resampling
        for i in range(self.N_PARTICLES):
            beta += random.random() * 2.0 * mw
            while beta > weights[index]:
                beta -= weights[index]
                index = (index + 1) % self.N_PARTICLES
            new_particles.append(self.particles[index])
        self.particles = new_particles


class RadialTracking(ParticleFilter):

    # NOTE: player may disappear.

    def __init__(self, n_particles, msg_time_thd=2./5, dead_rck_reset_thd=0.005):
        ParticleFilter.__init__(self, n_particles)
        self.pub = rospy.Publisher('angle', Float32, queue_size=1)
        self.sector_pub = rospy.Publisher(
            'sector_probabilities', SectorProbabilities, queue_size=1)
        self.sub = rospy.Subscriber(
            'people_tracked', PersonArray, self.callback)

        self.listener = tf.TransformListener()

        self.last_msg = PersonArray()
        self.last_person_msg = Person()
        self.msg_time_thd = msg_time_thd    # threshold for accepting the msg.
        self.dead_rck_counter = 0.0
        self.dead_rck_reset_thd = dead_rck_reset_thd
        self.is_on_dead_rck = False

        self.sectors_labels = [(0, "-180/135"),
                               (1, "-135/90"),
                               (2, "-90/45"),
                               (3, "-45/0"),
                               (4, "0/45"),
                               (5, "45/90"),
                               (6, "90/135"),
                               (7, "135/180")]

        self.plot_ordering = [4, 5, 6, 7, 0, 1, 2, 3]

    def callback(self, msg):
        """callback for PersonArray msgs"""
        max_confidence = float("-inf")
        best_track = None
        # get best person hipothesis
        for person in msg.people:
            if person.confidence > max_confidence:
                max_confidence = person.confidence
                best_track = person

        if max_confidence != float("-inf"):
            ps = PointStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.point.x = best_track.pose.position.x
            ps.point.y = best_track.pose.position.y

            velocity = PointStamped()
            velocity.header.frame_id = msg.header.frame_id
            velocity.header.stamp = msg.header.stamp
            velocity.point = best_track.velocity
            

            try:
                self.listener.waitForTransform(
                    "base_link", msg.header.frame_id, rospy.get_rostime(), rospy.Duration(1.0))
                ps = self.listener.transformPoint("base_link", ps)
                self.last_person_msg.pose.position = ps.point

                # NOTE  Updating last_person velocity as well
                # Velocity is a point in Person msg
                self.listener.waitForTransform(
                    "base_link", msg.header.frame_id, rospy.get_rostime(), rospy.Duration(1.0))
                vel = self.listener.transformPoint(
                    "base_link", velocity)
                self.last_person_msg.velocity = vel.point

                self.last_msg = msg
            except Exception as e:
                rospy.logerr(
                    "Not publishing people due to no transform from fixed_frame-->robot_frame! Details: {}".format(traceback.print_exc(e)))

    def check_msg_validity(self):
        """Checks whether to use the last msg to propagate particles
        Return:
            bool whether to use the las msg.
        """
        return not (abs((self.last_msg.header.stamp.to_sec() - rospy.get_rostime().to_sec())) > self.msg_time_thd)

    # def get_player_orientation(self):
    #     # Return orientation of the player wrt. the robot.
    #     # NOTE Redundant method as last_person_msg is already transformed wrt base_link in callback
    #     ps = PointStamped()
    #     ps.header.frame_id = self.last_msg.header.frame_id
    #     ps.header.stamp = self.last_msg.header.stamp
    #     ps.point.x = self.last_person_msg.pose.position.x
    #     ps.point.y = self.last_person_msg.pose.position.y

    #     try:
    #         self.listener.waitForTransform(
    #             "base_link", ps.header.frame_id, rospy.get_rostime(), rospy.Duration(1.0))
    #         ps = self.listener.transformPoint("base_link", ps)
    #         return np.arctan2(ps.point.y, ps.point.x)

    #     except Exception as e:
    #         rospy.logerr(
    #             "Not publishing people due to no transform from fixed_frame-->robot_frame! Details: {}".format(traceback.print_exc(e)))

    def calc_speed(self, velocity):
        return np.sqrt(velocity.x**2 + velocity.y**2)

    def calculate_displacement(self):
        return (self.last_msg.header.stamp.to_sec() - rospy.get_rostime().to_sec()) * self.calc_speed(self.last_person_msg.velocity)

    def calculate_time_diff(self):
        """ Returns the time difference between the current time and the last received message"""
        return (self.last_msg.header.stamp.to_sec() - rospy.get_rostime().to_sec())

    def propagate(self):
        """ Overriding Base function propagate method """
        # Use velocity to update particle positions
        vel = self.last_person_msg.velocity
        
        dt = self.calculate_time_diff()
        #observed_displacement = self.calculate_displacement()
        #orientation = self.get_player_orientation()
        for p in self.particles:
            # p.update(observed_displacement, orientation)
            p.update(vel, dt, mult_factor=10)

    def propagate_dead_reckoning(self):
        """ Propagate particles when on dead reckoning"""
    
        for p in self.particles:
            velocity = Point(np.random.normal(0, 1), np.random.normal(0, 1), 0)
            p.update(velocity, 1.0/10)

    def measurement_prob(self):
        """get measurement probability"""
        self.dead_rck_counter = 0
        weights = []
        obs_alpha = np.arctan2(
            self.last_person_msg.pose.position.y, self.last_person_msg.pose.position.x)
        rospy.logerr("Observed player angle:{:.2f}".format(
            np.mean(np.rad2deg(obs_alpha))
        ))
        for p in self.particles:
            # If the difference between angle exceeds 180 deg, take the complement
            diff = abs(p.alpha - obs_alpha)
            if diff > np.pi:
                diff = 2*np.pi - diff
            weights.append(1 - (diff / np.pi))
        return weights

    def measurement_prob_dead_reckoning(self):
        """get measurement probability when on dead_reckoning"""
        weights = []
        self.dead_rck_counter += 0.1
        for p in self.particles:
            weights.append(np.exp(-self.dead_rck_counter))
        return weights

    def get_sectors_probabilities(self, weights):
        """Returns the best orientation (angle sector) to rotate."""
        # NOTE using new convention -> sector number in [0,7]
        # sectors = dict(zip(range(int(-np.pi/0.785398) + 4, int(np.pi/0.785398) + 4),[0.0 for x in range(int(-np.pi/0.785398), int(np.pi/0.785398))]))
        # NOTE We can instead use an array an not worring about ordering later on
        sectors = [0] * 8  # TODO parametrize the number of sectors somewhere
        angles = []
        for i, p in enumerate(self.particles):
            #rospy.logerr("Particle angle:{}".format(p.alpha))
            angles.append(p.alpha)
            # Handling edge case where the angle is exactly 180 deg / pi rad
            if p.get_sector() == 8:
                sectors[7] += weights[i]
            else:
                sectors[p.get_sector()] += weights[i]

        rospy.logerr("Mean angle: {:.2f} degrees".format(
            np.mean(np.rad2deg(angles))))
        # NOTE sorting should not be needed anymore sorted(sectors.items(), key=operator.itemgetter(0))
        return sectors

    def publish_orientation(self, data):
        """Publishes the new orientation."""
        msg = Float32()
        msg.data = data
        self.pub.publish(msg)

    def publish_sector_probabilities(self, data):
        """Publish data for visualization with online_hist.py"""
        msg = SectorProbabilities()
        # NOTE this should no longer be needed. Anyhow it was sorting an already sorted dict
        # sorted_by_sector = sorted(data,key=operator.itemgetter(0))
        msg.labels = [i[1] for i in self.sectors_labels]
        msg.probabilities = data
        msg.plot_ordering = self.plot_ordering
        msg.on_dead_rck = self.is_on_dead_rck
        self.sector_pub.publish(msg)

    def run(self):
        # create particles
        self.particles = self.initialize()

        r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            weights = []

            # propagate particles and weight particles with measurement probability
            if self.check_msg_validity():
                self.is_on_dead_rck = False
                self.propagate()
                weights = self.measurement_prob()
            else:
                rospy.loginfo("On dead_reckoning")
                self.is_on_dead_rck = True
                self.propagate_dead_reckoning()
                weights = self.measurement_prob_dead_reckoning()
                sum_of_weights = sum(weights)
                rospy.logwarn("Sum of weights in dead:{}".format(sum_of_weights))
                if sum_of_weights < self.dead_rck_reset_thd:
                    self.dead_rck_counter = 0
                    self.particles = self.initialize()

            try:
                # normalize weights
                weights = self.normalize_weights(weights)

                # calcule probabilities
                probs = self.get_sectors_probabilities(weights)

                # resample
                self.resample(weights)

                max_value = max(probs)
                rospy.logerr("Vector probabilities{}".format(probs))#sorted(probs, key=operator.itemgetter(1))[0][0]

                # publish data
                self.publish_sector_probabilities(probs)
                self.publish_orientation(max_value)
            except Exception as exp:
                rospy.logerr(traceback.print_exc(exp))

            r.sleep()


if __name__ == '__main__':
    rospy.init_node('radial_tracker')
    tracker = RadialTracking(30)
    tracker.run()

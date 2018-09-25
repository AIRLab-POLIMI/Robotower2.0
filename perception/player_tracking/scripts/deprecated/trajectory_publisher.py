#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Path
from player_tracker.msg import PersonArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from std_msgs.msg import Header
from collections import defaultdict

class TrajectoryManager(object):

    def __init__(self, name):
        self.name = name
        self.trajectories = defaultdict(list)

        self._tfl = tf.TransformListener()
        self.counter = self.seq = 0
        self._publish_interval = 10.0
        
        rospy.loginfo("Creating %s/trajectories topic..." % name)
        
        #self._pub = rospy.Publisher(name+'/trajectories/complete', Trajectories, queue_size=10)
        self.path_pub = rospy.Publisher(name+'/trajectories', Path, queue_size=10)
        self.sub = rospy.Subscriber('players_tracked', PersonArray, self.callback)
        self._publish_rate = rospy.Rate(self._publish_interval)
        
        rospy.loginfo("human_trajectory is ready...")

    # construct trajectories message header
    def _construct_header(self):
        trajs = Trajectories()
        self.seq += 1
        trajs.header.seq = self.seq
        trajs.header.stamp = rospy.Time.now()
        trajs.header.frame_id = '/odom'
        return trajs

    def callback(self, player_tracks):
        converted_trackes = self.convert_frame(player_tracks)
        for id, pose in converted_trackes.iteritems():
            self.trajectories[id].append(pose)

    # publish based on online data from people_tracker
    def run(self):
        while not rospy.is_shutdown():

            # Publish path one at a time for allowing good
            # rviz visualization. Set Buffer on rviz.
            for id in self.trajectories.keys():
                trajs = self.trajectories[id]
                if len(trajs) != 0:
                    path = Path()
                    path.header = trajs[0].header
                    path.header.stamp = rospy.get_rostime()
                    path.poses = trajs
                    self.path_pub.publish(path)
            
            # trajs = []
            # for id in self.trajectories.keys():
            #     trajs += self.trajectories[id]
            
            # if len(trajs) != 0:
            #     path = Path()
            #     path.header = trajs[0].header
            #     path.header.stamp = rospy.get_rostime()
            #     path.poses = trajs
            #     self.path_pub.publish(path)

    def convert_frame(self, players_tracks):
        transformed_poses = dict()
        try:
            fid = players_tracks.header.frame_id
            
            for person in players_tracks.people:
                cpose = person.pose
                pose_stamped = PoseStamped(players_tracks.header, cpose)
                # Get the translation for this camera's frame to the world.
                # And apply it to all current detections.
                tpose = self._tfl.transformPose("/odom", pose_stamped)
                transformed_poses[person.id] = tpose
        except tf.Exception:
            rospy.logwarn("Transformation from %s to /map can not be done at the moment" % players_tracks.header.frame_id)
            # In case of a problem, just give empty world coordinates.
            return {}
        return transformed_poses

if __name__ == '__main__':
    rospy.init_node('players_trajectories')

    tp = TrajectoryManager(rospy.get_name())
    tp.run()

    rospy.spin()

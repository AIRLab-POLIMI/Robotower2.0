#!/usr/bin/python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
from perception_grid_match import PerceptionGridProjector, show_image

from math import atan2, degrees


class PerceptionGridFixer(object):
    def __init__(self):
        self.projector = PerceptionGridProjector('ORB')
        self.map = None
        self.map_sub = rospy.Subscriber('map', OccupancyGrid, self.map_callback)
        grid_topic = rospy.get_param('perception_grid_topic')
        self.percpetion_grid_sub = rospy.Subscriber(grid_topic, OccupancyGrid, self.grid_callback)

        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # TO FIX the problem of late map use map server!
        # map_servce = rospy.wait_for_service('static_map')
        # try:
        #     resp = map_servce()
        #     self.map_grid = resp.map
        #     rospy.logerr("Got Map!!")
        # except rospy.ServiceException as exc:
        #     print("Service did not process request: " + str(exc))

    def map_callback(self, map):
        map = self.extract_image(map, 4000)
        self.map_keyp, self.map_desc = self.projector.compute_descriptors(map)
        self.map_sub.unregister()
        self.map = map

    def pose_callback(self, pose):
        quaternion = (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = degrees(euler[2])

    def grid_callback(self, grid):
        if self.map is None:
            return      # return beacuse the map has not been received
        grid = self.extract_image(grid, 400)
        kp, desc = self.projector.compute_descriptors(grid)
        affine_matrix = self.projector.get_rotation_matrix(kp, desc, self.map_keyp, self.map_desc)

        # transformed_grid = self.projector.project_image(affine_matrix, grid)
        affine_matrix = np.transpose(affine_matrix)
        yaw = atan2(affine_matrix[1,0], affine_matrix[0,0])
        yaw = yaw if  yaw > 0 else 2 * np.pi + yaw
        rospy.loginfo( 'yaw from perception: %f' % (degrees(yaw)) )
        rospy.loginfo( 'amcl yaw:            %f' % (self.yaw) )

    def extract_image(self, grid_msg, side):    # returns an image where obstacles are white and free space is black
        raw_img = grid_msg.data
        raw_img = np.reshape(raw_img, (side, side))
        np.place(raw_img, raw_img < 0, 0)
        image = raw_img * (255.0 / raw_img.max())
        return image.astype(np.uint8)




if __name__ == '__main__':
    rospy.init_node('perception_grid_rototranslation', anonymous=True)
    perception_grid = PerceptionGridFixer()
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        r.sleep()

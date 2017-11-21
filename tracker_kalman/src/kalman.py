#!/usr/bin/env python

from __future__ import division
import rospy
import math
import tf
import os.path
import message_filters
from std_msgs.msg import String, Float64
from tracker_kalman.msg import PlayerPosition 
from kinect_tracker.msg import PlayerDistance, PlayerScreenAngle, PlayerPixelPosition
import collections
import numpy as np

RATE = 25

CAM_WIDTH = None
CAM_HEIGHT= None
WIDTH_FOV = None
HEIGHT_FOV= None
pixel_position = PlayerPixelPosition()
pixel_position.x = 0
pixel_position.y = 0

previous_angle = 0
previous_distance = 0
previous_time = 0
previous_ang_velocity = 0
previous_lin_velocity = 0

delta_t = (1./RATE)
delta_t2 = (delta_t**2)/2

buffer = collections.deque(maxlen=5)

#Setup publisher
#pub = rospy.Publisher('kalman/player_position', PlayerPosition, queue_size=10)
# TF-Broadcaster
br = tf.TransformBroadcaster()

x = np.array([0., 0., 0., 0., 0., 0.]).transpose()   # initial state (location and velocity)

u = np.array([0., 0., 0., 0., 0., 0.]).transpose()    # external motion (NO CONTROL INPUT ASSUMED)

#H = np.array([[1.,0.,0.,0.,0.,0.],             # measurement function 
#            [0.,1.,0.,0.,0.,0.],
#            [0.,0.,1.,0.,0.,0.],
#            [0.,0.,0.,1.,0.,0.],
#            [0.,0.,0.,0.,1.,0.],
#            [0.,0.,0.,0.,0.,1.]])  

H = np.array([[1.,0.,0.,0.,0.,0.],             # measurement function 6 states - 2 observed (angle and distance)
              [0.,1.,0.,0.,0.,0.]])  

I =  np.array([[1.,0.,0.,0.,0.,0.],            # identity matrix
              [0.,1.,0.,0.,0.,0.],
              [0.,0.,1.,0.,0.,0.],
              [0.,0.,0.,1.,0.,0.],
              [0.,0.,0.,0.,1.,0.],
              [0.,0.,0.,0.,0.,1.]])     

R =  np.array([[0.1,0.],                       # measurement uncertainty (2 uncorrelated measures with uncertainty)
               [0.,0.3]]) 

                               

P = np.array([[1.,0.,0.,0.,0.,0.],             # initial uncertainty
              [0.,1.,0.,0.,0.,0.],
              [0.,0.,1.,0.,0.,0.],
              [0.,0.,0.,1.,0.,0.],
              [0.,0.,0.,0.,1.,0.],
              [0.,0.,0.,0.,0.,1.]])            

                                                     # Transition Matrix
F = np.array([[1.,0.,delta_t,0.,delta_t2,0.],        # angle
             [0.,1.,0.,delta_t,0.,delta_t2],         # polar distance
             [0.,0.,1.,0.,delta_t,0.],               # angular speed
             [0.,0.,0.,1.,0.,delta_t],              # player velocity
             [0.,0.,0.,0.,1.,0.],                    # angular acceleration
             [0.,0.,0.,0.,0.,1.]])                   # player acceleration

Q =  np.array([[0.1,0.,0.,0.,0.,0.],         # process noise matrix
               [0.,0.1,0.,0.,0.,0.],
               [0.,0.,0.01,0.,0.,0.],
               [0.,0.,0.,0.01,0.,0.],
               [0.,0.,0.,0.,0.001,0.],
               [0.,0.,0.,0.,0.,0.001]])

def derivative(x1, x2):
    return (x2 - x1) / delta_t

def predict():
    global u,F,x,P

    # prediction
    x = np.dot(F, x) + u
    P = np.dot(np.dot(F, P), F.transpose()) + Q



def update(angle_msg, distance_msg):
    global u,H,F,R,I,x,P

    # measurement update
    Z = np.array([angle_msg, distance_msg])
    Y =  Z.transpose() - np.dot(H,x)
    S = np.dot(np.dot(H,P),H.transpose()) + R
    K = np.dot(np.dot(P,H.transpose()),np.linalg.inv(S))
    x = x + np.dot(K,Y)
    P = np.dot(I - np.dot(K,H),P)

def kalman_filter(angle, distance):
    #Implements the Kalman Filter function for measurement 
    #   update and prediction step
    update(angle, distance)
    predict()

def publishNewTF(new_angle, distance):
    # phi is the angular coordinate for the width
    global pixel_position

    rho = distance
    phi = new_angle          # (0.5 - pixel_position.x / CAM_WIDTH) * WIDTH_FOV
    theta = math.pi / 2 - (0.5 - pixel_position.y / CAM_HEIGHT) * HEIGHT_FOV
    
    x = rho * math.sin(theta) * math.cos(phi)
    y = rho * math.sin(theta) * math.sin(phi)
    z = rho * math.cos(theta)

    br.sendTransform((x,y,z), tf.transformations.quaternion_from_euler(0,0,0), rospy.Time.now(), "/player_filtered_link", "/kinect2_link")


def callback(PlayerScreenAngle, PlayerDistance, PlayerPixelPosition):
    global pixel_position
    pixel_position = PlayerPixelPosition
    buffer.append(PlayerDistance.distance)
    update(PlayerScreenAngle.angle, np.mean(buffer))


def statusNode():
    global CAM_WIDTH, CAM_HEIGHT, WIDTH_FOV , HEIGHT_FOV

    #Init node
    rospy.init_node('player_kalman_filter')
    rospy.loginfo('Starting Kalman filter...')

    pub_angle = rospy.Publisher('kalman/player_relative_angle', Float64, queue_size=10)

    CAM_WIDTH = rospy.get_param('/tracker_cam_width')
    CAM_HEIGHT= rospy.get_param('/tracker_cam_height')
    WIDTH_FOV = rospy.get_param('/tracker_width_fov')
    HEIGHT_FOV= rospy.get_param('/tracker_height_fov')

    # Setting up subscriber 
    angle_sub = message_filters.Subscriber('kinect2/player_relative_angle', PlayerScreenAngle)
    distance_sub = message_filters.Subscriber('kinect2/player_distance', PlayerDistance)
    pixel_position_sub = message_filters.Subscriber('kinect2/player_pixel_position', PlayerPixelPosition)

    # Setting up message filter
    ts = message_filters.ApproximateTimeSynchronizer([angle_sub,distance_sub, pixel_position_sub],1,1)
    ts.registerCallback(callback)

    rospy.loginfo('Looping...')

    rate = rospy.Rate(RATE)

    while not rospy.is_shutdown():
        
        predict()

        ####### PUBLISH PREDICT() RESULT ##########
        #rospy.loginfo("State: ")
        #print x
        #rospy.loginfo("Uncertainty:")
        #print P
        publishNewTF(x[0],x[1])
        new_angle = Float64()
        new_angle.data = x[0]
        pub_angle.publish(new_angle)



        # adjust rate and loo
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    statusNode()

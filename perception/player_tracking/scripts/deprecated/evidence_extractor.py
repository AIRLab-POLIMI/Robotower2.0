#!/usr/bin/env python2
import rospy
import numpy as np
from player_tracker.srv import LegProbabilityResponse, LegProbability
from sklearn.externals import joblib


class LegContextServer: 
    '''Process pairs of leg clusters and weight them by their
    probability of being a human using a generative model trained
    by data..''' 
    
    def __init__(self):

        self.model = joblib.load(rospy.get_param("leg_dist_model"))
        self.s = rospy.Service('get_human_leg_context', LegProbability, self.service_callback)
        rospy.spin()

    def service_callback(self, req):
        '''Uses self.model to calculate the probability of the pair being a person'''
        return LegProbabilityResponse(np.exp(self.model.score_samples(req.distance)))


if __name__ == '__main__':
    rospy.init_node('leg_context_server', anonymous=True)
    contexter = LegContextServer()
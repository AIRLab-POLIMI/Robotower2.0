#!/usr/bin/env python
import rospy
import tf
from social_analyzer import SocialAnalyzer


def main():
    """ The main ros loop"""
    #Init node
    social_analyzer = SocialAnalyzer()

    rospy.init_node('behavior_with_deception')

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():

        social_analyzer.set_position_agents()            # analyzing social situation

        if(social_analyzer.check_incoming_messages()):

            social_analyzer.get_targets()
            social_analyzer.get_outcome_matrices()
            social_analyzer.map_outcome_matrices_in_interdependence_space()
            social_analyzer.check_for_deception()

        rate.sleep()


if __name__ == '__main__':
    main()

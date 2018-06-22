#!/usr/bin/env python
import rospy
from behavior_control.srv import InvBlockInfo
from game_manager.msg import Towers
from matplotlib import pyplot as plt

__author__ = "Ewerton Oliveira"
__copyright__ = "Copyright 2018"
__credits__ = ["Ewerton Lopes"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Ewerton Oliveira"
__email__ = "ewerton.lopes@polimi.it"
__status__ = "Production"

class RiskAbility(object):

    def __init__(self, sample_time=5.0, save_to_file=False):
        rospy.init_node("risk_ability_plotter")
        self.always_reset = True           # whether to reset block counters uppon service request
        self.blocks = []
        self.atk_accuracy = []
        self.rate = rospy.Rate(10)
        self.sample_time = sample_time

        self.save_to_file = save_to_file
        self.file_handler = open("risk_ability_data.csv")

        self.service_name_to_call = "inv_block_info"

        self.NUM_TOWERS = rospy.get_param("/num_towers")

        rospy.loginfo("RISK-ABILITY PLOTTER: Waiting '%s' service to come up..." % self.service_name_to_call)
        rospy.wait_for_service(self.service_name_to_call)
        rospy.loginfo("RISK-ABILITY PLOTTER: making connection")
        self.srv_handler = rospy.ServiceProxy(self.service_name_to_call, InvBlockInfo)
        rospy.loginfo("RISK-ABILITY PLOTTER: connection with '%s' established!" % self.service_name_to_call)

        self.timer = rospy.Timer(rospy.Duration(self.sample_time),self._callback)
        
        self.sub = rospy.Subscriber('game_manager/towers/State', Towers,  self._tower_callback)

        self.tw_msg = None
    
    def talk_to_service(self):
        """Call the behavior_service
        Params
            req : a request msg
            resp : the response msg container
        Returns
            None
        """
        try:
            resp = self.srv_handler(self.always_reset)
            self.blocks.append(resp.inv_bloc_frq)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def _tower_callback(self, msg):
        """Towers Callback"""
        self.tw_msg = msg

    def calc_atk_accuracy(self):
        """Calculate the attack accuracy, which is defined as the ration between the
        number_LEDS_ON and NUM_presses."""

        if self.tw_msg is None:
            rospy.logerr_throttle(3, "RISK-ABILITY PLOTTER: No tower message as received so far. Skipping operation.")
        else:
            mean = []
            for i in range(1, self.NUM_TOWERS+1):
                tw_data = getattr(self.tw_msg, "tw"+str(i))
                mean.append(tw_data.press_accuracy)
            self.atk_accuracy.append(sum(mean)/float(self.NUM_TOWERS))
        
    def _callback(self, event):
        """Computes the risk-ability data"""
        self.calc_atk_accuracy()
        self.talk_to_service()
        if self.save_to_file:
            if len(self.atk_accuracy)!= 0 and len(self.atk_accuracy) == len(self.blocks):
                self.file_handler.write("{},{}\n".format(self.atk_accuracy[-1],self.blocks[-1]))

    def plot(self):
        while not rospy.is_shutdown():
            if not self.save_to_file:
                if len(self.atk_accuracy)!= 0 and len(self.atk_accuracy) == len(self.blocks):
                    rospy.logwarn("Inverse Block: {}".format(self.blocks[-1]))
                    plt.scatter(self.atk_accuracy, self.blocks, color='blue')
                    plt.ylabel("Attack_freq")
                    plt.xlabel("Inverse Block")
                    plt.draw()
                    plt.pause(0.00000000001)
                    self.rate.sleep()


if __name__ == "__main__":
    node = RiskAbility()
    node.plot()
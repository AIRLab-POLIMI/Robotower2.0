#!/usr/bin/env python
import sys
import rospy
import json
from behavior_control.srv import BehaviorParams

__author__ = "Ewerton Oliveira"
__copyright__ = "Copyright 2018"
__credits__ = ["Ewerton Lopes"]
__license__ = "MIT"
__version__ = "0.0.1"
__maintainer__ = "Ewerton Oliveira"
__email__ = "ewerton.lopes@polimi.it"
__status__ = "Production"


class Manager(object):
    """ The base class for the Difficulty Node
    It contains the function for service communication
    """
    def __init__(self, behavior_service='change_behavior'):
        rospy.loginfo("DIFFICULTY CONTROL: Waiting '%s' service to come up..." % behavior_service)
        rospy.wait_for_service(behavior_service)
        rospy.loginfo("DIFFICULTY CONTROL: making connection")
        self.srv_handler = rospy.ServiceProxy('change_behavior', BehaviorParams)
        rospy.loginfo("DIFFICULTY CONTROL: connection with '%s' established!" % behavior_service)
    
    def talk_to_service(self, max_sp, min_sp, bf_exp):
        """Call the behavior_service
        Params
            req : a request msg
            resp : the response msg container
        Returns
            None
        """
        try:
            resp = self.srv_handler(max_sp, min_sp, bf_exp)
            return resp
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class Console(Manager):
    """
    This derived class (from Manager) allows for manual change 
    of difficulty from the console. It is best used when launched
    apart from the main game_manager launch since it prints info
    to the standard output and weights for typed commands. For
    usage, follow the instructions on the screen after launching it.
    """
    def __init__(self):
        Manager.__init__(self)
        self.difficulties = self.get_difficulties()
        self.cmd_index = {1: 'very_easy',
                          2: 'easy',
                          3: 'medium',
                          4: 'hard',
                          5: 'very_hard',
                          6: 'print_params'}

    def get_difficulties(self):
        '''Retrieve difficulties from the param server
        Params
            None
        Returns
            None
        '''
        return rospy.get_param('difficulties')
    
    def cmd_error(self,cmd):
        """ Logs the command unknown error
        Params
            cmd :   the user (unknown) command
        Returns
            None
        """
        rospy.logerr('Command {} is unknown'.format(cmd))

    def read_in(self, str_msg):
        """ Read proof the input from the user
        Params
            str_msg :   the input call message
        Returns
            (A string)  : the chosen difficulty label 
            defined in the 'cmd_index' variable. Or 'None'
            in case the input is unknown.
        """
        cmd = raw_input(str_msg)
        try:
            return self.cmd_index[int(cmd)]
        except Exception as e:
            if cmd == 'Q' or cmd == 'q':
                exit(0)
            else:
                self.cmd_error(cmd)

    def change_difficulty(self, difficulty):
        max_sp = self.difficulties[difficulty]['max_speed']
        min_sp = self.difficulties[difficulty]['min_speed']
        bf_exp = self.difficulties[difficulty]['bf_exponent'] 
        resp = self.talk_to_service(max_sp,min_sp,bf_exp)
        if resp:
            rospy.loginfo("\nChanging to difficulty '{}' with parameters:\n{}".format(difficulty,
            resp))
            rospy.set_param("current_difficulty", difficulty)
            print

    def usage(self):
        """ Print options
        Params 
            None
        Return
            None
        """
        print '##############################'
        print '####  SET NEW DIFFICULTY  ####'
        print '##############################'

        print '\n@OPTIONS (key):'
        print '\t - very easy (1)'
        print '\t - easy      (2)'
        print '\t - medium    (3)'
        print '\t - hard      (4)'
        print '\t - very hard (5)'

        print '\n@SHOW PARAMETER SETTINGS:'
        print '\t - show difficulty settings (6)'
        print '\nPress Q to cleanly QUIT the node...\n'

    def print_params(self):
        """Print difficulty params"""
        print
        rospy.loginfo("Difficulty settings: \n{}".format(json.dumps(self.difficulties, indent=4)))
        print

    def run(self):
        """ The main loop """

        while not rospy.is_shutdown():
            self.usage()
            new_diff = self.read_in('Command: ') # this is a blocking statement
            if new_diff == 'print_params':
                self.print_params()
            elif new_diff is not None:
                self.change_difficulty(new_diff)

def run_console():
    """Runs the Console Version"""
    rospy.init_node('console_difficulty_control_node')
    c = Console()
    c.run()


if __name__ == '__main__':
    try:
        run_console()
    except rospy.ROSInterruptException:
        pass

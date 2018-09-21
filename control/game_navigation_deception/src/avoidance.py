#!/usr/bin/env python
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import rospy


class FuzzyAvoider:
    """
    TODO
    RR_LOWER_BOUND  = None  # Rear Right laser sector lower bound
    R_LOWER_BOUND = None    # Right laser sector lower bound
    FR_LOWER_BOUND = None   # Front Right laser sector lower bound
    FL_LOWER_BOUND = None   # Front Left laser sector lower bound
    L_LOWER_BOUND = None    # Left laser sector lower bound
    RL_LOWER_BOUND = None   # Rear Left laser sector lower bound
    """

    def __init__(self, rr_lower_bound, r_lower_bound, fr_lower_bound,
    fl_lower_bound, l_lower_bound, rl_lower_bound):
        
        self.RR_LOWER_BOUND = rr_lower_bound
        self.R_LOWER_BOUND  = r_lower_bound
        self.FR_LOWER_BOUND = fr_lower_bound
        self.FL_LOWER_BOUND = fl_lower_bound
        self.L_LOWER_BOUND  = l_lower_bound
        self.RL_LOWER_BOUND = rl_lower_bound

        # New Antecedent/Consequent objects hold universe variables and membership functions
        # Generate universe variables of category 1:
        # INPUTS (ranges minima): 
        # @ min_rear_right [0, 0.76] 
        # @ min_right [0, 0.76]
        # @ min_front_right [0, 0.76]
        # @ min_front_left [0, 0.76]
        # @ min_left [0, 0.76]
        # @ min_rear_left [0, 0.76]
        
        #self.min_rear_right = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min rear right')
        #self.min_right = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min right')
        #self.min_front_right  = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min front right')
        #self.min_front_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min front left') 
        #self.min_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min left')
        #self.min_rear_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min rear left')

        self.min_rear_right = ctrl.Antecedent(np.linspace(0, 11, 100), 'min rear right')
        self.min_right = ctrl.Antecedent(np.linspace(0, 11, 100), 'min right')
        self.min_front_right  = ctrl.Antecedent(np.linspace(0, 11, 100), 'min front right')
        self.min_front_left = ctrl.Antecedent(np.linspace(0, 11, 100), 'min front left') 
        self.min_left = ctrl.Antecedent(np.linspace(0, 11, 100), 'min left')
        self.min_rear_left = ctrl.Antecedent(np.linspace(0, 11, 100), 'min rear left')
        
        # Generate universe variables of category 2:
        # INPUTS (ranges minima indexes): 
        # @ index_rear_right [0, 1001] 
        # @ index_right [0, 1001] 
        # @ index_front_right [0, 1001] 
        # @ index_front_left [0, 1001] 
        # @ index_left [0, 1001] 
        # @ index_rear_left [0, 1001] 

        self.index_rear_right = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index rear right')
        self.index_right = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index right')
        self.index_front_right  = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index front right')
        self.index_front_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index front left')
        self.index_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index left')
        self.index_rear_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index rear left')
        
        
        # Generate universe variable of category 3:
        # INPUT (near goal condition):
        # @ near_goal [0, 1]
        
        self.near_goal = ctrl.Antecedent(np.linspace(0, 1.1, 100), 'near goal')

        
        # Generate universe variable of category 4:
        # OUTPUTS (velocities):
        # @ Vx [-0.5, 0.5]
        # @ Vy [-0.5, 0.5]
        
        self.Vx = ctrl.Consequent(np.linspace(-0.6, 0.6, 500), 'Vx out')
        self.Vy = ctrl.Consequent(np.linspace(-0.6, 0.6, 500), 'Vy out')

        
        # Generate fuzzy membership functions for variables of category 1
        self.min_rear_right['rr_close'] = fuzz.trapmf(self.min_rear_right.universe, [0, 0, 0.4, 0.45])
        self.min_rear_right['rr_far'] = fuzz.trapmf(self.min_rear_right.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_rear_right['rr_dontcare'] = fuzz.trapmf(self.min_rear_right.universe, [0.65, 0.7, 11, 11])
        self.min_right['r_close'] = fuzz.trapmf(self.min_right.universe, [0, 0, 0.4, 0.45])
        self.min_right['r_far'] = fuzz.trapmf(self.min_right.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_right['r_dontcare'] = fuzz.trapmf(self.min_right.universe, [0.65, 0.7, 11, 11])
        self.min_front_right['fr_close'] = fuzz.trapmf(self.min_front_right.universe, [0, 0, 0.4, 0.45])
        self.min_front_right['fr_far'] = fuzz.trapmf(self.min_front_right.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_front_right['fr_dontcare'] = fuzz.trapmf(self.min_front_right.universe, [0.65, 0.7, 11, 11])
        self.min_front_left['fl_close'] = fuzz.trapmf(self.min_front_left.universe, [0, 0, 0.4, 0.45])
        self.min_front_left['fl_far'] = fuzz.trapmf(self.min_front_left.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_front_left['fl_dontcare'] = fuzz.trapmf(self.min_front_left.universe, [0.65, 0.7, 11, 11])
        self.min_left['l_close'] = fuzz.trapmf(self.min_left.universe, [0, 0, 0.4, 0.45])
        self.min_left['l_far'] = fuzz.trapmf(self.min_left.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_left['l_dontcare'] = fuzz.trapmf(self.min_left.universe, [0.65, 0.7, 11, 11])
        self.min_rear_left['rl_close'] = fuzz.trapmf(self.min_rear_left.universe, [0, 0, 0.4, 0.45])
        self.min_rear_left['rl_far'] = fuzz.trapmf(self.min_rear_left.universe, [0.4, 0.45, 0.65, 0.7])
        self.min_rear_left['rl_dontcare'] = fuzz.trapmf(self.min_rear_left.universe, [0.65, 0.7, 11, 11])

        # Generate fuzzy membership functions for variables of category 2
        self.index_rear_right['rr_isrr'] = fuzz.trapmf(self.index_rear_right.universe, [0, 1, 100, 200])
        self.index_rear_right['rr_isr'] = fuzz.trapmf(self.index_rear_right.universe, [100, 200, 260, 360])
        self.index_rear_right['rr_isfr'] = fuzz.trapmf(self.index_rear_right.universe, [260, 361, 450, 550])
        self.index_rear_right['rr_isfl'] = fuzz.trapmf(self.index_rear_right.universe, [450, 550, 640, 740])
        self.index_rear_right['rr_isl'] = fuzz.trapmf(self.index_rear_right.universe, [640, 740, 800, 900])
        self.index_rear_right['rr_isrl'] = fuzz.trapmf(self.index_rear_right.universe, [800, 900, 1000, 1000])
        self.index_right['r_isrr'] = fuzz.trapmf(self.index_right.universe, [0, 1, 100, 200])
        self.index_right['r_isr'] = fuzz.trapmf(self.index_right.universe, [100, 200, 260, 360])
        self.index_right['r_isfr'] = fuzz.trapmf(self.index_right.universe, [260, 361, 450, 550])
        self.index_right['r_isfl'] = fuzz.trapmf(self.index_right.universe, [450, 550, 640, 740])
        self.index_right['r_isl'] = fuzz.trapmf(self.index_right.universe, [640, 740, 800, 900])
        self.index_right['r_isrl'] = fuzz.trapmf(self.index_right.universe, [800, 900, 1000, 1000])
        self.index_front_right['fr_isrr'] = fuzz.trapmf(self.index_front_right.universe, [0, 1, 100, 200])
        self.index_front_right['fr_isr'] = fuzz.trapmf(self.index_front_right.universe, [100, 200, 260, 360])
        self.index_front_right['fr_isfr'] = fuzz.trapmf(self.index_front_right.universe, [260, 361, 450, 550])
        self.index_front_right['fr_isfl'] = fuzz.trapmf(self.index_front_right.universe, [450, 550, 640, 740])
        self.index_front_right['fr_isl'] = fuzz.trapmf(self.index_front_right.universe, [640, 740, 800, 900])
        self.index_front_right['fr_isrl'] = fuzz.trapmf(self.index_front_right.universe, [800, 900, 1000, 1000])   
        self.index_front_left['fl_isrr'] = fuzz.trapmf(self.index_front_left.universe, [0, 1, 100, 200])
        self.index_front_left['fl_isr'] = fuzz.trapmf(self.index_front_left.universe, [100, 200, 260, 360])
        self.index_front_left['fl_isfr'] = fuzz.trapmf(self.index_front_left.universe, [260, 361, 450, 550])
        self.index_front_left['fl_isfl'] = fuzz.trapmf(self.index_front_left.universe, [450, 550, 640, 740])
        self.index_front_left['fl_isl'] = fuzz.trapmf(self.index_front_left.universe, [640, 740, 800, 900])
        self.index_front_left['fl_isrl'] = fuzz.trapmf(self.index_front_left.universe, [800, 900, 1000, 1000]) 
        self.index_left['l_isrr'] = fuzz.trapmf(self.index_left.universe, [0, 1, 100, 200])
        self.index_left['l_isr'] = fuzz.trapmf(self.index_left.universe, [100, 200, 260, 360])
        self.index_left['l_isfr'] = fuzz.trapmf(self.index_left.universe, [260, 361, 450, 550])
        self.index_left['l_isfl'] = fuzz.trapmf(self.index_left.universe, [450, 550, 640, 740])
        self.index_left['l_isl'] = fuzz.trapmf(self.index_left.universe, [640, 740, 800, 900])
        self.index_left['l_isrl'] = fuzz.trapmf(self.index_left.universe, [800, 900, 1000, 1000])
        self.index_rear_left['rl_isrr'] = fuzz.trapmf(self.index_rear_left.universe, [0, 1, 100, 200])
        self.index_rear_left['rl_isr'] = fuzz.trapmf(self.index_rear_left.universe, [100, 200, 260, 360])
        self.index_rear_left['rl_isfr'] = fuzz.trapmf(self.index_rear_left.universe, [260, 361, 450, 550])
        self.index_rear_left['rl_isfl'] = fuzz.trapmf(self.index_rear_left.universe, [450, 550, 640, 740])
        self.index_rear_left['rl_isl'] = fuzz.trapmf(self.index_rear_left.universe, [640, 740, 800, 900])
        self.index_rear_left['rl_isrl'] = fuzz.trapmf(self.index_rear_left.universe, [800, 900, 1000, 1000])

        # Generate fuzzy membership functions for variable of category 3
        self.near_goal['false'] = fuzz.trimf(self.near_goal.universe, [0 ,0 ,0])
        self.near_goal['true'] = fuzz.trimf(self.near_goal.universe, [1 ,1 ,1])

        # Generate fuzzy membership functions for variable of category 4
        # final output to be sent to ROS:
        # U1_f = U1 + Vy
        # U2_f = U2 - Vx
        
        # delta_fuzz define the width of the singleton, for smaller values the controller will tends to return the full value of the considered outputs MMF
        # WARNING: values of delta_fuzz that are too small will cause the controller to stop working, always check that output MMF functions are consistent
        self.delta_fuzz = 0.05

        # Vx output definition (singletons):
        self.Vx['-0.5'] = fuzz.trimf(self.Vx.universe, [-0.5 - self.delta_fuzz ,-0.5 ,-0.5 + self.delta_fuzz])
        self.Vx['-0.4'] = fuzz.trimf(self.Vx.universe, [-0.4 - self.delta_fuzz ,-0.4 ,-0.4 + self.delta_fuzz])
        self.Vx['-0.3'] = fuzz.trimf(self.Vx.universe, [-0.3 - self.delta_fuzz ,-0.3 ,-0.3 + self.delta_fuzz])
        self.Vx['-0.2'] = fuzz.trimf(self.Vx.universe, [-0.2 - self.delta_fuzz ,-0.2 ,-0.2 + self.delta_fuzz])
        self.Vx['-0.1'] = fuzz.trimf(self.Vx.universe, [-0.1 - self.delta_fuzz ,-0.1 ,-0.1 + self.delta_fuzz])
        self.Vx['vxstop'] = fuzz.trimf(self.Vx.universe, [ - self.delta_fuzz ,0 , + self.delta_fuzz])
        self.Vx['0.1'] = fuzz.trimf(self.Vx.universe, [0.1 - self.delta_fuzz ,0.1 ,0.1 + self.delta_fuzz])
        self.Vx['0.2'] = fuzz.trimf(self.Vx.universe, [0.2 - self.delta_fuzz ,0.2 ,0.2 + self.delta_fuzz])
        self.Vx['0.3'] = fuzz.trimf(self.Vx.universe, [0.3 - self.delta_fuzz ,0.3 ,0.3 + self.delta_fuzz])
        self.Vx['0.4'] = fuzz.trimf(self.Vx.universe, [0.4 - self.delta_fuzz ,0.4 ,0.4 + self.delta_fuzz])
        self.Vx['0.5'] = fuzz.trimf(self.Vx.universe, [0.5 - self.delta_fuzz ,0.5 ,0.5 + self.delta_fuzz])
        # Vy output definition (singletons):
        self.Vy['-0.5'] = fuzz.trimf(self.Vy.universe, [-0.5 - self.delta_fuzz ,-0.5 ,-0.5 + self.delta_fuzz])
        self.Vy['-0.4'] = fuzz.trimf(self.Vy.universe, [-0.4 - self.delta_fuzz ,-0.4 ,-0.4 + self.delta_fuzz])
        self.Vy['-0.3'] = fuzz.trimf(self.Vy.universe, [-0.3 - self.delta_fuzz ,-0.3 ,-0.3 + self.delta_fuzz])
        self.Vy['-0.2'] = fuzz.trimf(self.Vy.universe, [-0.2 - self.delta_fuzz ,-0.2 ,-0.2 + self.delta_fuzz])
        self.Vy['-0.1'] = fuzz.trimf(self.Vy.universe, [-0.1 - self.delta_fuzz ,-0.1 ,-0.1 + self.delta_fuzz])
        self.Vy['vystop'] = fuzz.trimf(self.Vy.universe, [- self.delta_fuzz ,0 , + self.delta_fuzz])
        self.Vy['0.1'] = fuzz.trimf(self.Vy.universe, [0.1 - self.delta_fuzz ,0.1 ,0.1 + self.delta_fuzz])
        self.Vy['0.2'] = fuzz.trimf(self.Vy.universe, [0.2 - self.delta_fuzz ,0.2 ,0.2 + self.delta_fuzz])
        self.Vy['0.3'] = fuzz.trimf(self.Vy.universe, [0.3 - self.delta_fuzz ,0.3 ,0.3 + self.delta_fuzz])
        self.Vy['0.4'] = fuzz.trimf(self.Vy.universe, [0.4 - self.delta_fuzz ,0.4 ,0.4 + self.delta_fuzz])
        self.Vy['0.5'] = fuzz.trimf(self.Vy.universe, [0.5 - self.delta_fuzz ,0.5 ,0.5 + self.delta_fuzz])

        # Visualize membership
        #Vy['-0.5'].view()
        #plt.show()
        

        
        # FUZZY RULES SECTION
        
        self.rule1 = ctrl.Rule(self.min_front_left['fl_close'] | self.min_front_right['fr_close'], (self.Vx['vxstop'], self.Vy['-0.5']))
        self.rule2 = ctrl.Rule(self.min_front_left['fl_far'] & self.min_front_right['fr_far'] & self.near_goal['false'], (self.Vx['vxstop'],self.Vy['-0.4']))
        self.rule3 = ctrl.Rule(self.min_rear_left['rl_close'] & self.index_rear_left['rl_isrl'] & self.near_goal['false'], (self.Vx['0.4'],self.Vy['0.5']))
        self.rule4 = ctrl.Rule(self.min_rear_left['rl_close'] & self.index_rear_left['rl_isl'] & self.near_goal['false'], (self.Vx['0.5'],self.Vy['0.4']))
        self.rule5 = ctrl.Rule(self.min_front_left['fl_far'] & self.index_rear_left['rl_isrl'] & self.near_goal['false'], (self.Vx['0.3'],self.Vy['0.4']))
        self.rule6 = ctrl.Rule(self.min_rear_left['rl_far'] & self.index_rear_left['rl_isl'] & self.near_goal['false'], (self.Vx['0.4'],self.Vy['0.3']))
        self.rule7 = ctrl.Rule(self.min_rear_right['rr_close'] & self.index_rear_left['rl_isrr'] & self.near_goal['false'], (self.Vx['-0.4'],self.Vy['0.5']))
        self.rule8 = ctrl.Rule(self.min_rear_right['rr_close'] & self.index_rear_right['rr_isr'] & self.near_goal['false'], (self.Vx['-0.5'],self.Vy['0.4']))
        self.rule9 = ctrl.Rule(self.min_rear_right['rr_far'] & self.index_rear_right['rr_isrr'] & self.near_goal['false'], (self.Vx['-0.3'],self.Vy['0.4']))
        self.rule10 = ctrl.Rule(self.min_rear_right['rr_far'] & self.index_rear_right['rr_isr'] & self.near_goal['false'], (self.Vx['-0.4'],self.Vy['0.3']))
        self.rule11 = ctrl.Rule(self.min_left['l_close'] & self.index_left['l_isl'] & self.near_goal['false'], (self.Vx['0.5'],self.Vy['vystop']))
        self.rule12 = ctrl.Rule(self.min_left['l_close'] & self.index_left['l_isrl'] & self.near_goal['false'], (self.Vx['0.5'],self.Vy['0.2']))
        self.rule13 = ctrl.Rule(self.min_left['l_close'] & self.index_left['l_isfl'] & self.near_goal['false'], (self.Vx['0.5'],self.Vy['0.2']))
        self.rule14 = ctrl.Rule(self.min_left['l_far'] & self.index_left['l_isl'] & self.near_goal['false'], (self.Vx['0.4'],self.Vy['vystop']))
        self.rule15 = ctrl.Rule(self.min_left['l_far'] & self.index_left['l_isrl'] & self.near_goal['false'], (self.Vx['0.4'],self.Vy['0.1']))
        self.rule16 = ctrl.Rule(self.min_left['l_far'] & self.index_left['l_isfl'] & self.near_goal['false'], (self.Vx['0.4'],self.Vy['-0.1']))
        self.rule17 = ctrl.Rule(self.min_right['r_close'] & self.index_right['r_isr'] & self.near_goal['false'], (self.Vx['-0.5'],self.Vy['vystop'])) 
        self.rule18 = ctrl.Rule(self.min_right['r_close'] & self.index_right['r_isrr'] & self.near_goal['false'], (self.Vx['-0.5'],self.Vy['0.2']))
        self.rule19 = ctrl.Rule(self.min_right['r_close'] & self.index_right['r_isfr'] & self.near_goal['false'], (self.Vx['-0.5'],self.Vy['-0.2']))
        self.rule20 = ctrl.Rule(self.min_right['r_far'] & self.index_right['r_isr'] & self.near_goal['false'], (self.Vx['-0.4'],self.Vy['vystop']))
        self.rule21 = ctrl.Rule(self.min_right['r_far'] & self.index_right['r_isrr'] & self.near_goal['false'], (self.Vx['-0.4'],self.Vy['0.1']))
        self.rule22 = ctrl.Rule(self.min_right['r_far'] & self.index_right['r_isfr'] & self.near_goal['false'], (self.Vx['-0.4'],self.Vy['-0.1']))
        self.rule23 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_far'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isfl'] & self.near_goal['false'], (self.Vx['vxstop'],self.Vy['-0.3']))
        self.rule24 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_far'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isl'] & self.near_goal['false'], (self.Vx['0.3'],self.Vy['-0.1'])) 
        self.rule25 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_far'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isfr'] & self.near_goal['false'], (self.Vx['-0.3'],self.Vy['-0.1']))
        self.rule26 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_close'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isfl'] & self.near_goal['false'], (self.Vx['vxstop'],self.Vy['-0.3']))
        self.rule27 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_close'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isl'] & self.near_goal['false'], (self.Vx['0.3'],self.Vy['-0.2'])) 
        self.rule28 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_close'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_left['fl_isfr'] & self.near_goal['false'], (self.Vx['-0.3'],self.Vy['-0.2']))
        self.rule29 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_far'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isfl'] & self.near_goal['false'], (self.Vx['vxstop'],self.Vy['-0.3']))
        self.rule30 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_far'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isl'] & self.near_goal['false'], (self.Vx['-0.3'],self.Vy['-0.1'])) 
        self.rule31 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_far'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isfr'] & self.near_goal['false'], (self.Vx['0.3'],self.Vy['-0.1']))
        self.rule32 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_close'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isfl'] & self.near_goal['false'], (self.Vx['vxstop'],self.Vy['-0.3']))
        self.rule33 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_close'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isl'] & self.near_goal['false'], (self.Vx['-0.3'],self.Vy['-0.2'])) 
        self.rule34 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_close'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.index_front_right['fr_isfr'] & self.near_goal['false'], (self.Vx['0.3'],self.Vy['-0.2'])) 
        # rule to approach final target HERE:
        self.rule35 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_left['l_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['vxstop'],self.Vy['-0.5'])) 
        self.rule36 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & (~self.min_rear_right['rr_dontcare']) & self.near_goal['true'], (self.Vx['0.4'],self.Vy['-0.4'])) 
        self.rule37 = ctrl.Rule(~self.min_rear_left['rl_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['-0.4'],self.Vy['0.4'])) 
        self.rule38 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & (~self.min_left['l_dontcare']) & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['-0.4'],self.Vy['-0.1'])) 
        self.rule39 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & (~self.min_right['r_dontcare']) & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['0.4'],self.Vy['-0.1']))
        self.rule40 = ctrl.Rule(~self.min_rear_left['rl_dontcare'] & self.min_front_left['fl_dontcare'] & self.min_front_right['fr_dontcare'] & (~self.min_rear_right['rr_dontcare']) & self.near_goal['true'], (self.Vx['vxstop'],self.Vy['-0.4'])) 
        self.rule41 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & self.min_front_left['fl_dontcare'] & (~self.min_right['r_dontcare']) & (~self.min_rear_right['rr_dontcare']) & self.near_goal['true'], (self.Vx['0.4'],self.Vy['-0.3'])) 
        self.rule42 = ctrl.Rule(~self.min_rear_left['rl_dontcare'] & (~self.min_left['l_dontcare']) & self.min_front_left['fl_dontcare'] & self.min_right['r_dontcare'] & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['-0.4'],self.Vy['-0.3']))
        # if player is in front of the robot in near target area ??
        # self.rule43 = ctrl.Rule(self.min_rear_left['rl_dontcare'] & (~self.min_front_left['fl_dontcare']) | (~self.min_front_right['fr_dontcare']) & self.min_rear_right['rr_dontcare'] & self.near_goal['true'], (self.Vx['vxstop'],self.Vy['-0.5'])) 
        


        # Now that we have our rules defined, we can simply create a control system via:
        self.fuzzyavoider_ctrl = ctrl.ControlSystem([self.rule1,self.rule2,self.rule3,self.rule4,self.rule5,self.rule6,self.rule7,self.rule8,self.rule9,self.rule10,self.rule11,self.rule12,self.rule13,self.rule14,self.rule15,self.rule16,self.rule17,self.rule18,self.rule19,self.rule20,
        self.rule21,self.rule22,self.rule23,self.rule24,self.rule25,self.rule26,self.rule27,self.rule28,self.rule29,self.rule30,self.rule31,self.rule32,self.rule33,self.rule34,self.rule35,self.rule36,self.rule37,self.rule38,self.rule39,self.rule40,self.rule41,self.rule42])
        # In order to simulate this control system, we will create a ControlSystemSimulation
    
        self.fuzzyAvoider = ctrl.ControlSystemSimulation(self.fuzzyavoider_ctrl)
        


    def avoidObstacle(self, rear_right, right, front_right, front_left, left, rear_left, is_near_goal):
        '''
        TODO
        Fuzzy Avoider: this function will perform a dynamic obstacle (player) avoidance using fuzzy rules
        INPUTS:
            @ is_near_goal: boolean variable that become TRUE if the robot is close to a targeted tower
            Tuples containing minimum and index of the minimum for each sector  ---->>    
            @ rear_right
            @ right
            @ front_right
            @ front_left
            @ left
            @ rear_left 
        OUTPUTS: 
            @ avoiderVel: linear xy-velocity comands (robot frame) 
        '''

        # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
        # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
        
        # Laser minimum detected distance for each sector:
        #fuzzyAvoider.input['min rear right'] = 0.6
        #fuzzyAvoider.input['min right'] = 0.2
        #fuzzyAvoider.input['min front right'] = 0.6
        #fuzzyAvoider.input['min front left'] = 0.6
        #fuzzyAvoider.input['min left'] = 0.7
        #fuzzyAvoider.input['min rear left'] = 0.7
        # index of the minimum detected distance for each sector:
        #fuzzyAvoider.input['index rear right'] = 32
        #fuzzyAvoider.input['index right'] = 100
        #fuzzyAvoider.input['index front right'] = 200
        #fuzzyAvoider.input['index front left'] = 361
        #fuzzyAvoider.input['index left'] = 550
        #fuzzyAvoider.input['index rear left'] = 740
        #fuzzyAvoider.input['near goal'] = 0

        # Laser minimum detected distance for each sector:
        self.fuzzyAvoider.input['min rear right'] = rear_right[0]
        self.fuzzyAvoider.input['min right'] = right[0]
        self.fuzzyAvoider.input['min front right'] = front_right[0]
        self.fuzzyAvoider.input['min front left'] = front_left[0]
        self.fuzzyAvoider.input['min left'] = left[0]
        self.fuzzyAvoider.input['min rear left'] = rear_left[0]
        # index of the minimum detected distance for each sector:
        self.fuzzyAvoider.input['index rear right'] = rear_right[1] + self.RR_LOWER_BOUND
        self.fuzzyAvoider.input['index right'] = right[1] + self.R_LOWER_BOUND
        self.fuzzyAvoider.input['index front right'] = front_right[1] + self.FR_LOWER_BOUND
        self.fuzzyAvoider.input['index front left'] = front_left[1] + self.FL_LOWER_BOUND
        self.fuzzyAvoider.input['index left'] = left[1] + self.L_LOWER_BOUND
        self.fuzzyAvoider.input['index rear left'] = rear_left[1] + self.RL_LOWER_BOUND
        self.fuzzyAvoider.input['near goal'] = is_near_goal


        #rospy.logwarn("rr: {}, r: {}, fr: {}, fl: {}, l: {}, rl: {}".format(rear_right, right, front_right, front_left, left, rear_left))

        # Crunch the numbers
        self.fuzzyAvoider.compute()
        
        # print out the numerical results
        #print "Vx_out: {}".format(self.fuzzyAvoider.output['Vx out'])
        #print "Vy_out: {}".format(self.fuzzyAvoider.output['Vy out'])
        # plotting and testing the controller
        #self.Vx.view(sim=self.fuzzyAvoider)
        #plt.title('Vx Fuzzy decision')
        #self.Vy.view(sim=self.fuzzyAvoider)
        #plt.title('Vy Fuzzy decision')
        #plt.show()
        
        
        # Generate the output linear.cmd.vel in robot frame
        return (self.fuzzyAvoider.output['Vy out'], -self.fuzzyAvoider.output['Vx out'])

if __name__ == "__main__":
    example()
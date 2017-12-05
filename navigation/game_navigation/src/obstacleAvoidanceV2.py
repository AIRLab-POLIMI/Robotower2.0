#!/usr/bin/env python
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

def fuzzyAvoider(unsmoothed_vel):
    """Implements obstacle avoidance using Fuzzy logic
    TODO 
     Step 1 - write similar code to example(). plot the membership functions
     checking whether they are similar to MATLAB.
     Step 2 - write rules.
    """
    pass

def example():
    # New Antecedent/Consequent objects hold universe variables and membership functions
    #quality = ctrl.Antecedent(np.arange(0, 11, 1), 'quality')
    #service = ctrl.Antecedent(np.arange(0, 11, 1), 'service')
    #tip = ctrl.Consequent(np.arange(0, 26, 1), 'tip')

    """
    Generate universe variables of category 1:
    INPUTS (ranges minima): 
    @ min_rear_right [0, 0.76] 
    @ min_right [0, 0.76]
    @ min_front_right [0, 0.76]
    @ min_front_left [0, 0.76]
    @ min_left [0, 0.76]
    @ min_rear_left [0, 0.76]
    """
    min_rear_right = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min rear right')
    min_right = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min right')
    min_front_right  = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min front right')
    min_front_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min front left') 
    min_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min left')
    min_rear_left = ctrl.Antecedent(np.linspace(0, 0.76, 100), 'min rear left')

    """
    Generate universe variables of category 2:
    INPUTS (ranges minima indexes): 
    @ index_rear_right [0, 1001] 
    @ index_right [0, 1001] 
    @ index_front_right [0, 1001] 
    @ index_front_left [0, 1001] 
    @ index_left [0, 1001] 
    @ index_rear_left [0, 1001] 
    """
    index_rear_right = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index rear right')
    index_right = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index right')
    index_front_right  = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index front right')
    index_front_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index front left')
    index_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index left')
    index_rear_left = ctrl.Antecedent(np.linspace(0, 1001, 500), 'index rear left')
    
    """
    Generate universe variable of category 3:
    INPUT (near goal condition):
    @ near_goal [0, 1]
    """
    near_goal = ctrl.Antecedent(np.linspace(0, 1.1, 100), 'near goal')

    """
    Generate universe variable of category 4:
    OUTPUTS (velocities):
    @ Vx [-0.5, 0.5]
    @ Vy [-0.5, 0.5]
    """
    Vx = ctrl.Consequent(np.linspace(-0.6, 0.6, 500), 'Vx out')
    Vy = ctrl.Consequent(np.linspace(-0.6, 0.6, 500), 'Vy out')

    
    # Generate fuzzy membership functions for variables of category 1
    min_rear_right['rr_close'] = fuzz.trapmf(min_rear_right.universe, [0, 0, 0.4, 0.45])
    min_rear_right['rr_far'] = fuzz.trapmf(min_rear_right.universe, [0.4, 0.45, 0.65, 0.7])
    min_rear_right['rr_dontcare'] = fuzz.trapmf(min_rear_right.universe, [0.65, 0.7, 0.75, 0.75])
    min_right['r_close'] = fuzz.trapmf(min_right.universe, [0, 0, 0.4, 0.45])
    min_right['r_far'] = fuzz.trapmf(min_right.universe, [0.4, 0.45, 0.65, 0.7])
    min_right['r_dontcare'] = fuzz.trapmf(min_right.universe, [0.65, 0.7, 0.75, 0.75])
    min_front_right['fr_close'] = fuzz.trapmf(min_front_right.universe, [0, 0, 0.4, 0.45])
    min_front_right['fr_far'] = fuzz.trapmf(min_front_right.universe, [0.4, 0.45, 0.65, 0.7])
    min_front_right['fr_dontcare'] = fuzz.trapmf(min_front_right.universe, [0.65, 0.7, 0.75, 0.75])
    min_front_left['fl_close'] = fuzz.trapmf(min_front_left.universe, [0, 0, 0.4, 0.45])
    min_front_left['fl_far'] = fuzz.trapmf(min_front_left.universe, [0.4, 0.45, 0.65, 0.7])
    min_front_left['fl_dontcare'] = fuzz.trapmf(min_front_left.universe, [0.65, 0.7, 0.75, 0.75])
    min_left['l_close'] = fuzz.trapmf(min_left.universe, [0, 0, 0.4, 0.45])
    min_left['l_far'] = fuzz.trapmf(min_left.universe, [0.4, 0.45, 0.65, 0.7])
    min_left['l_dontcare'] = fuzz.trapmf(min_left.universe, [0.65, 0.7, 0.75, 0.75])
    min_rear_left['rl_close'] = fuzz.trapmf(min_rear_left.universe, [0, 0, 0.4, 0.45])
    min_rear_left['rl_far'] = fuzz.trapmf(min_rear_left.universe, [0.4, 0.45, 0.65, 0.7])
    min_rear_left['rl_dontcare'] = fuzz.trapmf(min_rear_left.universe, [0.65, 0.7, 0.75, 0.75])

    # Generate fuzzy membership functions for variables of category 2
    index_rear_right['rr_isrr'] = fuzz.trapmf(index_rear_right.universe, [0, 1, 100, 200])
    index_rear_right['rr_isr'] = fuzz.trapmf(index_rear_right.universe, [100, 200, 260, 360])
    index_rear_right['rr_isfr'] = fuzz.trapmf(index_rear_right.universe, [260, 361, 450, 550])
    index_rear_right['rr_isfl'] = fuzz.trapmf(index_rear_right.universe, [450, 550, 640, 740])
    index_rear_right['rr_isl'] = fuzz.trapmf(index_rear_right.universe, [640, 740, 800, 900])
    index_rear_right['rr_isrl'] = fuzz.trapmf(index_rear_right.universe, [800, 900, 1000, 1000])
    index_right['r_isrr'] = fuzz.trapmf(index_right.universe, [0, 1, 100, 200])
    index_right['r_isr'] = fuzz.trapmf(index_right.universe, [100, 200, 260, 360])
    index_right['r_isfr'] = fuzz.trapmf(index_right.universe, [260, 361, 450, 550])
    index_right['r_isfl'] = fuzz.trapmf(index_right.universe, [450, 550, 640, 740])
    index_right['r_isl'] = fuzz.trapmf(index_right.universe, [640, 740, 800, 900])
    index_right['r_isrl'] = fuzz.trapmf(index_right.universe, [800, 900, 1000, 1000])
    index_front_right['fr_isrr'] = fuzz.trapmf(index_front_right.universe, [0, 1, 100, 200])
    index_front_right['fr_isr'] = fuzz.trapmf(index_front_right.universe, [100, 200, 260, 360])
    index_front_right['fr_isfr'] = fuzz.trapmf(index_front_right.universe, [260, 361, 450, 550])
    index_front_right['fr_isfl'] = fuzz.trapmf(index_front_right.universe, [450, 550, 640, 740])
    index_front_right['fr_isl'] = fuzz.trapmf(index_front_right.universe, [640, 740, 800, 900])
    index_front_right['fr_isrl'] = fuzz.trapmf(index_front_right.universe, [800, 900, 1000, 1000])   
    index_front_left['fl_isrr'] = fuzz.trapmf(index_front_left.universe, [0, 1, 100, 200])
    index_front_left['fl_isr'] = fuzz.trapmf(index_front_left.universe, [100, 200, 260, 360])
    index_front_left['fl_isfr'] = fuzz.trapmf(index_front_left.universe, [260, 361, 450, 550])
    index_front_left['fl_isfl'] = fuzz.trapmf(index_front_left.universe, [450, 550, 640, 740])
    index_front_left['fl_isl'] = fuzz.trapmf(index_front_left.universe, [640, 740, 800, 900])
    index_front_left['fl_isrl'] = fuzz.trapmf(index_front_left.universe, [800, 900, 1000, 1000]) 
    index_left['l_isrr'] = fuzz.trapmf(index_left.universe, [0, 1, 100, 200])
    index_left['l_isr'] = fuzz.trapmf(index_left.universe, [100, 200, 260, 360])
    index_left['l_isfr'] = fuzz.trapmf(index_left.universe, [260, 361, 450, 550])
    index_left['l_isfl'] = fuzz.trapmf(index_left.universe, [450, 550, 640, 740])
    index_left['l_isl'] = fuzz.trapmf(index_left.universe, [640, 740, 800, 900])
    index_left['l_isrl'] = fuzz.trapmf(index_left.universe, [800, 900, 1000, 1000])
    index_rear_left['rl_isrr'] = fuzz.trapmf(index_rear_left.universe, [0, 1, 100, 200])
    index_rear_left['rl_isr'] = fuzz.trapmf(index_rear_left.universe, [100, 200, 260, 360])
    index_rear_left['rl_isfr'] = fuzz.trapmf(index_rear_left.universe, [260, 361, 450, 550])
    index_rear_left['rl_isfl'] = fuzz.trapmf(index_rear_left.universe, [450, 550, 640, 740])
    index_rear_left['rl_isl'] = fuzz.trapmf(index_rear_left.universe, [640, 740, 800, 900])
    index_rear_left['rl_isrl'] = fuzz.trapmf(index_rear_left.universe, [800, 900, 1000, 1000])

    # Generate fuzzy membership functions for variable of category 3
    near_goal['false'] = fuzz.trimf(near_goal.universe, [0 ,0 ,0])
    near_goal['true'] = fuzz.trimf(near_goal.universe, [1 ,1 ,1])

    # Generate fuzzy membership functions for variable of category 4
    Vx['-0.5'] = fuzz.trimf(Vx.universe, [-0.55 ,-0.5 ,-0.45])
    Vx['-0.4'] = fuzz.trimf(Vx.universe, [-0.45 ,-0.4 ,-0.35])
    Vx['-0.3'] = fuzz.trimf(Vx.universe, [-0.35 ,-0.3 ,-0.25])
    Vx['-0.2'] = fuzz.trimf(Vx.universe, [-0.25 ,-0.2 ,-0.15])
    Vx['-0.1'] = fuzz.trimf(Vx.universe, [-0.15 ,-0.1 ,-0.05])
    Vx['vxstop'] = fuzz.trimf(Vx.universe, [-0.05 ,0 , 0.05])
    Vx['0.1'] = fuzz.trimf(Vy.universe, [0.05 ,0.1 ,0.15])
    Vx['0.2'] = fuzz.trimf(Vy.universe, [0.15 ,0.2 ,0.25])
    Vx['0.3'] = fuzz.trimf(Vy.universe, [0.25 ,0.3 ,0.35])
    Vx['0.4'] = fuzz.trimf(Vy.universe, [0.35 ,0.4 ,0.45])
    Vx['0.5'] = fuzz.trimf(Vy.universe, [0.45 ,0.5 ,0.55])

    Vy['-0.5'] = fuzz.trimf(Vy.universe, [-0.55 ,-0.5 ,-0.45])
    Vy['-0.4'] = fuzz.trimf(Vy.universe, [-0.45 ,-0.4 ,-0.35])
    Vy['-0.3'] = fuzz.trimf(Vy.universe, [-0.35 ,-0.3 ,-0.25])
    Vy['-0.2'] = fuzz.trimf(Vy.universe, [-0.25 ,-0.2 ,-0.15])
    Vy['-0.1'] = fuzz.trimf(Vy.universe, [-0.15 ,-0.1 ,-0.05])
    Vy['vystop'] = fuzz.trimf(Vy.universe, [-0.05 ,0 ,0.05])
    Vy['0.1'] = fuzz.trimf(Vy.universe, [0.05 ,0.1 ,0.15])
    Vy['0.2'] = fuzz.trimf(Vy.universe, [0.15 ,0.2 ,0.25])
    Vy['0.3'] = fuzz.trimf(Vy.universe, [0.25 ,0.3 ,0.35])
    Vy['0.4'] = fuzz.trimf(Vy.universe, [0.35 ,0.4 ,0.45])
    Vy['0.5'] = fuzz.trimf(Vy.universe, [0.45 ,0.5 ,0.55])

    # Visualize membership
    #Vy['-0.5'].view()
    #plt.show()
    

    """
    FUZZY RULES SECTION
    """
    rule1 = ctrl.Rule(min_front_left['fl_close'] & min_front_right['fr_close'], (Vx['vxstop'], Vy['-0.4']))
    rule2 = ctrl.Rule(min_front_left['fl_far'] & min_front_right['fr_far'] & near_goal['false'], (Vx['0.2'],Vy['-0.1']))
    rule3 = ctrl.Rule(min_rear_left['rl_close'] & index_rear_left['rl_isrl'] & near_goal['false'], (Vx['0.4'],Vy['0.5']))
    rule4 = ctrl.Rule(min_rear_left['rl_close'] & index_rear_left['rl_isl'] & near_goal['false'], (Vx['0.5'],Vy['0.4']))
    rule5 = ctrl.Rule(min_front_left['fl_far'] & index_rear_left['rl_isrl'] & near_goal['false'], (Vx['0.3'],Vy['0.4']))
    rule6 = ctrl.Rule(min_rear_left['rl_far'] & index_rear_left['rl_isl'] & near_goal['false'], (Vx['0.4'],Vy['0.3']))
    rule7 = ctrl.Rule(min_rear_right['rr_close'] & index_rear_left['rl_isrr'] & near_goal['false'], (Vx['-0.4'],Vy['0.5']))
    rule8 = ctrl.Rule(min_rear_right['rr_close'] & index_rear_right['rr_isr'] & near_goal['false'], (Vx['-0.5'],Vy['0.4']))
    rule9 = ctrl.Rule(min_rear_right['rr_far'] & index_rear_right['rr_isrr'] & near_goal['false'], (Vx['-0.3'],Vy['0.4']))
    rule10 = ctrl.Rule(min_rear_right['rr_far'] & index_rear_right['rr_isr'] & near_goal['false'], (Vx['-0.4'],Vy['0.3']))
    rule11 = ctrl.Rule(min_left['l_close'] & index_left['l_isl'] & near_goal['false'], (Vx['0.5'],Vy['vystop']))
    rule12 = ctrl.Rule(min_left['l_close'] & index_left['l_isrl'] & near_goal['false'], (Vx['0.5'],Vy['0.2']))
    rule13 = ctrl.Rule(min_left['l_close'] & index_left['l_isfl'] & near_goal['false'], (Vx['0.5'],Vy['0.2']))
    rule14 = ctrl.Rule(min_left['l_far'] & index_left['l_isl'] & near_goal['false'], (Vx['0.4'],Vy['vystop']))
    rule15 = ctrl.Rule(min_left['l_far'] & index_left['l_isrl'] & near_goal['false'], (Vx['0.4'],Vy['0.1']))
    rule16 = ctrl.Rule(min_left['l_far'] & index_left['l_isfl'] & near_goal['false'], (Vx['0.4'],Vy['-0.1']))
    rule17 = ctrl.Rule(min_right['r_close'] & index_right['r_isr'] & near_goal['false'], (Vx['-0.5'],Vy['vystop'])) 
    rule18 = ctrl.Rule(min_right['r_close'] & index_right['r_isrr'] & near_goal['false'], (Vx['-0.5'],Vy['0.2']))
    rule19 = ctrl.Rule(min_right['r_close'] & index_right['r_isfr'] & near_goal['false'], (Vx['-0.5'],Vy['-0.2']))
    rule20 = ctrl.Rule(min_right['r_far'] & index_right['r_isr'] & near_goal['false'], (Vx['-0.4'],Vy['vystop']))
    rule21 = ctrl.Rule(min_right['r_far'] & index_right['r_isrr'] & near_goal['false'], (Vx['-0.4'],Vy['0.1']))
    rule22 = ctrl.Rule(min_right['r_far'] & index_right['r_isfr'] & near_goal['false'], (Vx['-0.4'],Vy['-0.1']))
    rule23 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_far'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isfl'] & near_goal['false'], (Vx['vxstop'],Vy['-0.3']))
    rule24 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_far'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isl'] & near_goal['false'], (Vx['0.3'],Vy['-0.1'])) 
    rule25 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_far'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isfr'] & near_goal['false'], (Vx['-0.3'],Vy['-0.1']))
    rule26 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_close'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isfl'] & near_goal['false'], (Vx['vxstop'],Vy['-0.3']))
    rule27 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_close'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isl'] & near_goal['false'], (Vx['0.3'],Vy['-0.2'])) 
    rule28 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_close'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_left['fl_isfr'] & near_goal['false'], (Vx['-0.3'],Vy['-0.2']))
    rule29 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_far'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isfl'] & near_goal['false'], (Vx['vxstop'],Vy['-0.3']))
    rule30 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_far'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isl'] & near_goal['false'], (Vx['-0.3'],Vy['-0.1'])) 
    rule31 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_far'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isfr'] & near_goal['false'], (Vx['0.3'],Vy['-0.1']))
    rule32 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_close'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isfl'] & near_goal['false'], (Vx['vxstop'],Vy['-0.3']))
    rule33 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_close'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isl'] & near_goal['false'], (Vx['-0.3'],Vy['-0.2'])) 
    rule34 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_close'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & index_front_right['fr_isfr'] & near_goal['false'], (Vx['0.3'],Vy['-0.2'])) 
    # rule to approach final target HERE:
    rule35 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_left['l_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & near_goal['true'], (Vx['vxstop'],Vy['-0.5'])) 
    rule36 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & (~min_rear_right['rr_dontcare']) & near_goal['true'], (Vx['0.4'],Vy['-0.4'])) 
    rule37 = ctrl.Rule(~min_rear_left['rl_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & min_rear_right['rr_dontcare'] & near_goal['true'], (Vx['-0.4'],Vy['0.4'])) 
    rule38 = ctrl.Rule(min_rear_left['rl_dontcare'] & (~min_left['l_dontcare']) & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & min_rear_right['rr_dontcare'] & near_goal['true'], (Vx['-0.4'],Vy['-0.1'])) 
    rule39 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & (~min_right['r_dontcare']) & min_rear_right['rr_dontcare'] & near_goal['true'], (Vx['0.4'],Vy['-0.1']))
    rule40 = ctrl.Rule(~min_rear_left['rl_dontcare'] & min_front_left['fl_dontcare'] & min_front_right['fr_dontcare'] & (~min_rear_right['rr_dontcare']) & near_goal['true'], (Vx['vxstop'],Vy['-0.4'])) 
    rule41 = ctrl.Rule(min_rear_left['rl_dontcare'] & min_front_left['fl_dontcare'] & (~min_right['r_dontcare']) & (~min_rear_right['rr_dontcare']) & near_goal['true'], (Vx['0.4'],Vy['-0.3'])) 
    rule42 = ctrl.Rule(~min_rear_left['rl_dontcare'] & (~min_left['l_dontcare']) & min_front_left['fl_dontcare'] & min_right['r_dontcare'] & min_rear_right['rr_dontcare'] & near_goal['true'], (Vx['-0.4'],Vy['-0.3']))
 


    # Now that we have our rules defined, we can simply create a control system via:
    fuzzyavoider_ctrl = ctrl.ControlSystem([rule1,rule2,rule3,rule4,rule5,rule6,rule7,rule8,rule9,rule10,rule11,rule12,rule13,rule14,rule15,rule16,rule17,rule18,rule19,rule20,
    rule21,rule22,rule23,rule24,rule25,rule26,rule27,rule28,rule29,rule30,rule31,rule32,rule33,rule34,rule35,rule36,rule37,rule38,rule39,rule40,rule41,rule42])
    # In order to simulate this control system, we will create a ControlSystemSimulation
   
    fuzzyAvoider = ctrl.ControlSystemSimulation(fuzzyavoider_ctrl)


    # Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
    # Note: if you like passing many inputs all at once, use .inputs(dict_of_data)
    fuzzyAvoider.input['min rear right'] = 0.7
    fuzzyAvoider.input['min right'] = 0.7
    fuzzyAvoider.input['min front right'] = 0.1
    fuzzyAvoider.input['min front left'] = 0.1
    fuzzyAvoider.input['min left'] = 0.7
    fuzzyAvoider.input['min rear left'] = 0.7
    
    fuzzyAvoider.input['index rear right'] = 32
    fuzzyAvoider.input['index right'] = 100
    fuzzyAvoider.input['index front right'] = 200
    fuzzyAvoider.input['index front left'] = 361
    fuzzyAvoider.input['index left'] = 550
    fuzzyAvoider.input['index rear left'] = 740
    fuzzyAvoider.input['near goal'] = 0

    # Crunch the numbers
    fuzzyAvoider.compute()

    print "Vx_out: {}".format(fuzzyAvoider.output['Vx out'])
    print "Vy_out: {}".format(fuzzyAvoider.output['Vy out'])
    
    Vx.view(sim=fuzzyAvoider)
    plt.title('Vx Fuzzy decision')
    Vy.view(sim=fuzzyAvoider)
    plt.title('Vy Fuzzy decision')
    plt.show()

if __name__ == "__main__":
    example()
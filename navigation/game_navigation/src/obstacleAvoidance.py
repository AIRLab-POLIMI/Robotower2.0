#!/usr/bin/env python
import numpy as np
import skfuzzy as fuzz
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
    min_rear_right = np.linspace(0, 0.76, 100)
    min_right = np.linspace(0, 0.76, 100)
    min_front_right  = np.linspace(0, 0.76, 100)
    min_front_left = np.linspace(0, 0.76, 100) 
    min_left = np.linspace(0, 0.76, 100)
    min_rear_left = np.linspace(0, 0.76, 100)
    
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
    index_rear_right = np.linspace(0, 1001, 500)
    index_right = np.linspace(0, 1001, 500)
    index_front_right  = np.linspace(0, 1001, 500)
    index_front_left = np.linspace(0, 1001, 500) 
    index_left = np.linspace(0, 1001, 500)
    index_rear_left = np.linspace(0, 1001, 500)
    
    """
    Generate universe variable of category 3:
    INPUT (near goal condition):
    @ near_goal [0, 1]
    """
    near_goal = np.linspace(0, 1, num=100)

    """
    Generate universe variable of category 4:
    OUTPUTS (velocities):
    @ Vx [-0.5, 0.5]
    @ Vy [-0.5, 0.5]
    """
    Vx = np.linspace(-0.5, 1, num=500)
    Vy = np.linspace(-0.5, 1, num=500)

    # Generate fuzzy membership functions for variables of category 1
    rr_close = fuzz.trapmf(min_rear_right, [0, 0, 0.4, 0.45])
    rr_far = fuzz.trapmf(min_rear_right, [0.4, 0.45, 0.65, 0.7])
    rr_dontcare = fuzz.trapmf(min_rear_right, [0.65, 0.7, 0.75, 0.75])
    r_close = fuzz.trapmf(min_right, [0, 0, 0.4, 0.45])
    r_far = fuzz.trapmf(min_right, [0.4, 0.45, 0.65, 0.7])
    r_dontcare = fuzz.trapmf(min_right, [0.65, 0.7, 0.75, 0.75])
    fr_close = fuzz.trapmf(min_front_right, [0, 0, 0.4, 0.45])
    fr_far = fuzz.trapmf(min_front_right, [0.4, 0.45, 0.65, 0.7])
    fr_dontcare = fuzz.trapmf(min_front_right, [0.65, 0.7, 0.75, 0.75])
    fl_close = fuzz.trapmf(min_front_left, [0, 0, 0.4, 0.45])
    fl_far = fuzz.trapmf(min_front_left, [0.4, 0.45, 0.65, 0.7])
    fl_dontcare = fuzz.trapmf(min_front_left, [0.65, 0.7, 0.75, 0.75])
    l_close = fuzz.trapmf(min_left, [0, 0, 0.4, 0.45])
    l_far = fuzz.trapmf(min_left, [0.4, 0.45, 0.65, 0.7])
    l_dontcare = fuzz.trapmf(min_left, [0.65, 0.7, 0.75, 0.75])
    rl_close = fuzz.trapmf(min_rear_left, [0, 0, 0.4, 0.45])
    rl_far = fuzz.trapmf(min_rear_left, [0.4, 0.45, 0.65, 0.7])
    rl_dontcare = fuzz.trapmf(min_rear_left, [0.65, 0.7, 0.75, 0.75])

    # Generate fuzzy membership functions for variables of category 2
    rr_isrr = fuzz.trapmf(index_rear_right, [0, 1, 100, 200])
    rr_isr = fuzz.trapmf(index_rear_right, [100, 200, 260, 360])
    rr_isfr = fuzz.trapmf(index_rear_right, [260, 361, 450, 550])
    rr_isfl = fuzz.trapmf(index_rear_right, [450, 550, 640, 740])
    rr_isl = fuzz.trapmf(index_rear_right, [640, 740, 800, 900])
    rr_isrl = fuzz.trapmf(index_rear_right, [800, 900, 1000, 1000])
    r_isrr = fuzz.trapmf(index_right, [0, 1, 100, 200])
    r_isr = fuzz.trapmf(index_right, [100, 200, 260, 360])
    r_isfr = fuzz.trapmf(index_right, [260, 361, 450, 550])
    r_isfl = fuzz.trapmf(index_right, [450, 550, 640, 740])
    r_isl = fuzz.trapmf(index_right, [640, 740, 800, 900])
    r_isrl = fuzz.trapmf(index_right, [800, 900, 1000, 1000])
    fr_isrr = fuzz.trapmf(index_front_right, [0, 1, 100, 200])
    fr_isr = fuzz.trapmf(index_front_right, [100, 200, 260, 360])
    fr_isfr = fuzz.trapmf(index_front_right, [260, 361, 450, 550])
    fr_isfl = fuzz.trapmf(index_front_right, [450, 550, 640, 740])
    fr_isl = fuzz.trapmf(index_front_right, [640, 740, 800, 900])
    fr_isrl = fuzz.trapmf(index_front_right, [800, 900, 1000, 1000])   
    fl_isrr = fuzz.trapmf(index_front_left, [0, 1, 100, 200])
    fl_isr = fuzz.trapmf(index_front_left, [100, 200, 260, 360])
    fl_isfr = fuzz.trapmf(index_front_left, [260, 361, 450, 550])
    fl_isfl = fuzz.trapmf(index_front_left, [450, 550, 640, 740])
    fl_isl = fuzz.trapmf(index_front_left, [640, 740, 800, 900])
    fl_isrl = fuzz.trapmf(index_front_left, [800, 900, 1000, 1000]) 
    l_isrr = fuzz.trapmf(index_left, [0, 1, 100, 200])
    l_isr = fuzz.trapmf(index_left, [100, 200, 260, 360])
    l_isfr = fuzz.trapmf(index_left, [260, 361, 450, 550])
    l_isfl = fuzz.trapmf(index_left, [450, 550, 640, 740])
    l_isl = fuzz.trapmf(index_left, [640, 740, 800, 900])
    l_isrl = fuzz.trapmf(index_left, [800, 900, 1000, 1000])
    rl_isrr = fuzz.trapmf(index_rear_left, [0, 1, 100, 200])
    rl_isr = fuzz.trapmf(index_rear_left, [100, 200, 260, 360])
    rl_isfr = fuzz.trapmf(index_rear_left, [260, 361, 450, 550])
    rl_isfl = fuzz.trapmf(index_rear_left, [450, 550, 640, 740])
    rl_isl = fuzz.trapmf(index_rear_left, [640, 740, 800, 900])
    rl_isrl = fuzz.trapmf(index_rear_left, [800, 900, 1000, 1000])     
    
    # Generate fuzzy membership functions for variable of category 3
    near_goal_false = fuzz.trimf(near_goal, [0 ,0 ,0])
    near_goal_true = fuzz.trimf(near_goal, [1 ,1 ,1])
    
    # Generate fuzzy membership functions for variable of category 4
    n_vx1 = fuzz.trimf(Vx, [-0.5 ,-0.5 ,-0.5])
    n_vx2 = fuzz.trimf(Vx, [-0.4 ,-0.4 ,-0.4])
    n_vx3 = fuzz.trimf(Vx, [-0.3 ,-0.3 ,-0.3])
    n_vx4 = fuzz.trimf(Vx, [-0.2 ,-0.2 ,-0.2])
    n_vx5 = fuzz.trimf(Vx, [-0.1 ,-0.1 ,-0.1])
    vxstop = fuzz.trimf(Vx, [0 ,0 ,0])
    p_vx1 = fuzz.trimf(Vx, [0.1 ,0.1 ,0.1])
    p_vx2 = fuzz.trimf(Vx, [0.2 ,0.2 ,0.2])
    p_vx3 = fuzz.trimf(Vx, [0.3 ,0.3 ,0.3])
    p_vx4 = fuzz.trimf(Vx, [0.4 ,0.4 ,0.4])
    p_vx5 = fuzz.trimf(Vx, [0.5 ,0.5 ,0.5])

    n_vy1 = fuzz.trimf(Vy, [-0.5 ,-0.5 ,-0.5])
    n_vy2 = fuzz.trimf(Vy, [-0.4 ,-0.4 ,-0.4])
    n_vy3 = fuzz.trimf(Vy, [-0.3 ,-0.3 ,-0.3])
    n_vy4 = fuzz.trimf(Vy, [-0.2 ,-0.2 ,-0.2])
    n_vy5 = fuzz.trimf(Vy, [-0.1 ,-0.1 ,-0.1])
    vystop = fuzz.trimf(Vy, [0 ,0 ,0])
    p_vy1 = fuzz.trimf(Vy, [0.1 ,0.1 ,0.1])
    p_vy2 = fuzz.trimf(Vy, [0.2 ,0.2 ,0.2])
    p_vy3 = fuzz.trimf(Vy, [0.3 ,0.3 ,0.3])
    p_vy4 = fuzz.trimf(Vy, [0.4 ,0.4 ,0.4])
    p_vy5 = fuzz.trimf(Vy, [0.5 ,0.5 ,0.5])

    """
    # Visualize these universes and membership functions (CATEGORY 1)
    fig, (ax0, ax1, ax2, ax3, ax4, ax5) = plt.subplots(nrows=6, figsize=(8, 9))
    
    ax0.plot(min_rear_right, rr_close, 'b', linewidth=1.5, label='Close')
    ax0.plot(min_rear_right, rr_far, 'g', linewidth=1.5, label='Far')
    ax0.plot(min_rear_right, rr_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax0.set_title('Min Rear Right sector MMF')
    ax0.legend()

    ax1.plot(min_right, r_close, 'b', linewidth=1.5, label='Close')
    ax1.plot(min_right, r_far, 'g', linewidth=1.5, label='Far')
    ax1.plot(min_right, r_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax1.set_title('Min Right sector MMF')
    ax1.legend()

    ax2.plot(min_front_right, fr_close, 'b', linewidth=1.5, label='Close')
    ax2.plot(min_front_right, fr_far, 'g', linewidth=1.5, label='Far')
    ax2.plot(min_front_right, fr_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax2.set_title('Min Front Right sector MMF')
    ax2.legend()

    ax3.plot(min_front_left, fr_close, 'b', linewidth=1.5, label='Close')
    ax3.plot(min_front_left, fr_far, 'g', linewidth=1.5, label='Far')
    ax3.plot(min_front_left, fr_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax3.set_title('Min Front Left sector MMF')
    ax3.legend()

    ax4.plot(min_left, l_close, 'b', linewidth=1.5, label='Close')
    ax4.plot(min_left, l_far, 'g', linewidth=1.5, label='Far')
    ax4.plot(min_left, l_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax4.set_title('Min Left sector MMF')
    ax4.legend()

    ax5.plot(min_rear_left, rl_close, 'b', linewidth=1.5, label='Close')
    ax5.plot(min_rear_left, rl_far, 'g', linewidth=1.5, label='Far')
    ax5.plot(min_rear_left, rl_dontcare, 'r', linewidth=1.5, label='Dontcare')
    ax5.set_title('Min Rear Left sector MMF')
    ax5.legend()

    # Turn off top/right axes
    for ax in (ax0, ax1, ax2, ax3, ax4, ax5):
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

    # Visualize these universes and membership functions (CATEGORY 2)

    fig, (ax0, ax1, ax2, ax3, ax4, ax5) = plt.subplots(nrows=6, figsize=(8, 9))
    
    ax0.plot(index_rear_right, rr_isrr, linewidth=1.5, label='is RR')
    ax0.plot(index_rear_right, rr_isr, linewidth=1.5, label='is R')
    ax0.plot(index_rear_right, rr_isfr, linewidth=1.5, label='is FR')
    ax0.plot(index_rear_right, rr_isfl, linewidth=1.5, label='is FL')
    ax0.plot(index_rear_right, rr_isl, linewidth=1.5, label='is L')
    ax0.plot(index_rear_right, rr_isrl, linewidth=1.5, label='is RL')
    ax0.set_title('Index Rear Right sector MMF')
    ax0.legend()

    ax1.plot(index_right, r_isrr, linewidth=1.5, label='is RR')
    ax1.plot(index_right, r_isr, linewidth=1.5, label='is R')
    ax1.plot(index_right, r_isfr, linewidth=1.5, label='is FR')
    ax1.plot(index_right, r_isfl, linewidth=1.5, label='is FL')
    ax1.plot(index_right, r_isl, linewidth=1.5, label='is L')
    ax1.plot(index_right, r_isrl, linewidth=1.5, label='is RL')
    ax1.set_title('Index Right sector MMF')
    ax1.legend()

    ax2.plot(index_front_right, fr_isrr, linewidth=1.5, label='is RR')
    ax2.plot(index_front_right, fr_isr, linewidth=1.5, label='is R')
    ax2.plot(index_front_right, fr_isfr, linewidth=1.5, label='is FR')
    ax2.plot(index_front_right, fr_isfl, linewidth=1.5, label='is FL')
    ax2.plot(index_front_right, fr_isl, linewidth=1.5, label='is L')
    ax2.plot(index_front_right, fr_isrl, linewidth=1.5, label='is RL')
    ax2.set_title('Index Front Right sector MMF')
    ax2.legend()

    ax3.plot(index_front_left, fl_isrr, linewidth=1.5, label='is RR')
    ax3.plot(index_front_left, fl_isr, linewidth=1.5, label='is R')
    ax3.plot(index_front_left, fl_isfr, linewidth=1.5, label='is FR')
    ax3.plot(index_front_left, fl_isfl, linewidth=1.5, label='is FL')
    ax3.plot(index_front_left, fl_isl, linewidth=1.5, label='is L')
    ax3.plot(index_front_left, fl_isrl, linewidth=1.5, label='is RL')
    ax3.set_title('Index Front Left sector MMF')
    ax3.legend()

    ax4.plot(index_left, l_isrr, linewidth=1.5, label='is RR')
    ax4.plot(index_left, l_isr, linewidth=1.5, label='is R')
    ax4.plot(index_left, l_isfr, linewidth=1.5, label='is FR')
    ax4.plot(index_left, l_isfl, linewidth=1.5, label='is FL')
    ax4.plot(index_left, l_isl, linewidth=1.5, label='is L')
    ax4.plot(index_left, l_isrl, linewidth=1.5, label='is RL')
    ax4.set_title('Index Left sector MMF')
    ax4.legend()

    ax5.plot(index_rear_left, rl_isrr, linewidth=1.5, label='is RR')
    ax5.plot(index_rear_left, rl_isr, linewidth=1.5, label='is R')
    ax5.plot(index_rear_left, rl_isfr, linewidth=1.5, label='is FR')
    ax5.plot(index_rear_left, rl_isfl, linewidth=1.5, label='is FL')
    ax5.plot(index_rear_left, rl_isl, linewidth=1.5, label='is L')
    ax5.plot(index_rear_left, rl_isrl, linewidth=1.5, label='is RL')
    ax5.set_title('Index Rear Left sector MMF')
    ax5.legend()

    # Turn off top/right axes
    for ax in (ax0, ax1, ax2, ax3, ax4, ax5):
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

    # Visualize these universes and membership functions (CATEGORY 3)
    fig, (ax0) = plt.subplots(nrows=1, figsize=(8, 9))

    ax0.plot(near_goal, near_goal_false, linewidth=1.5, label='FALSE')
    ax0.plot(near_goal, near_goal_true, linewidth=1.5, label='TRUE')
    ax0.set_title('Near Goal Condition')
    ax0.legend()

    # Visualize these universes and membership functions (CATEGORY 2)

    fig, (ax0, ax1) = plt.subplots(nrows=2, figsize=(8, 9))
    
    ax0.plot(Vx, n_vx1, linewidth=1, label='-0.5m/s')
    ax0.plot(Vx, n_vx2, linewidth=1, label='-0.4m/s')
    ax0.plot(Vx, n_vx3, linewidth=1, label='-0.3m/s')
    ax0.plot(Vx, n_vx4, linewidth=1, label='-0.2m/s')
    ax0.plot(Vx, n_vx5, linewidth=1, label='-0.1m/s')
    ax0.plot(Vx, vxstop,linewidth=1, label='stop')
    ax0.plot(Vx, p_vx1, linewidth=1, label='0.1m/s')
    ax0.plot(Vx, p_vx2, linewidth=1, label='0.2m/s')
    ax0.plot(Vx, p_vx3, linewidth=1, label='0.3m/s')
    ax0.plot(Vx, p_vx4, linewidth=1, label='0.4m/s')
    ax0.plot(Vx, p_vx5, linewidth=1, color= "black", label='0.5m/s')
    ax0.set_title('Vx Output MMF')
    ax0.legend()

    ax1.plot(Vy, n_vy1, linewidth=1, label='-0.5m/s')
    ax1.plot(Vy, n_vy2, linewidth=1, label='-0.4m/s')
    ax1.plot(Vy, n_vy3, linewidth=1, label='-0.3m/s')
    ax1.plot(Vy, n_vy4, linewidth=1, label='-0.2m/s')
    ax1.plot(Vy, n_vy5, linewidth=1, label='-0.1m/s')
    ax1.plot(Vy, vystop,linewidth=1, label='stop')
    ax1.plot(Vy, p_vy1, linewidth=1, label='0.1m/s')
    ax1.plot(Vy, p_vy2, linewidth=1, label='0.2m/s')
    ax1.plot(Vy, p_vy3, linewidth=1, label='0.3m/s')
    ax1.plot(Vy, p_vy4, linewidth=1, label='0.4m/s')
    ax1.plot(Vy, p_vy5, linewidth=1, color= "black", label='0.5m/s')
    ax1.set_title('Vy Output MMF')
    ax1.legend()

    # Turn off top/right axes
    for ax in (ax0, ax1):
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.get_xaxis().tick_bottom()
        ax.get_yaxis().tick_left()

    plt.tight_layout()

    plt.show()
    """
    
    
    # Generate Fuzzy Rules
    rule1 = fuzz.Rule(min_front_left['fl_close'],Vx['vxstop'],Vy['n_vy2'])
    
    
    # Generate the Fuzzy Avoider Control System
    fuzzy_avoider = fuzz.ControlSystem([rule1])
    # View the whole control system
    fuzzy_avoider.view()
    avoider = fuzz.ControlSystemSimulation(fuzzy_avoider)
    # Give an input to the controller
    avoider.input['min_front_left'] = 0.2
    # Crunch the numbers
    avoider.compute()
    # Output the variables as a dictionary
    print avoider.output
    avoider.print_state()
    #set the outputs
    Vx.view(sim=avoider)
    Vy.view(sim=avoider)   

if __name__ == "__main__":
    example()
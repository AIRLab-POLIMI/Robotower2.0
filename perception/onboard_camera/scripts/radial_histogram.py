#!/usr/bin/env python
from plotwindow import PlotWindow


import rospy
import pandas as pd
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import numpy as np
import matplotlib.lines as mlines
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from scipy.stats import norm
from collections import deque
from std_msgs.msg import Int8
from sensor_msgs.msg import Range
from radial_tracking.msg import SectorProbabilities
from player_tracker.msg import Leg, LegArray 

class OnlineHist(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=500
    self.values= deque(maxlen=self.window_size)  #numpy.zeros((self.window_size))
    self.index=0
    self.draw_counter =0
    self.paused = False

    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("sector_probabilities", SectorProbabilities, self.plotResults, queue_size = 1 )
    self.leg_array_subscriber = rospy.Subscriber("/detected_leg_clusters", LegArray, self.legCallback, queue_size = 1 )
    
    self.pauseButton.clicked.connect(self.pauseClicked)
    self.resetButton.clicked.connect(self.resetClicked)
    
  def pauseClicked(self):
    if self.paused:        
       self.paused = False
    else:
       self.paused = True
  
  def resetClicked(self):
    self.draw_counter =0  
    self.values.clear()
    self.index=0       
    self.paused = False

  def plotResults(self, data):       
    #self.axes.set_autoscaley_on(True)

    if self.index==self.window_size-1:
      self.index=0
    else:
      self.index=self.index+1
    #self.values.append(round(data.range,3))

    self.draw_counter = self.draw_counter + 1
    
    if self.draw_counter > 10 and not self.paused:
        self.draw_counter = 0
        
        self.axes.clear()      

        width = np.arange(0,2*np.pi, np.deg2rad(30))
        #width = 0.35       # the width of the bars
        #bins = self.axes.bar(range(len(data.probabilities)), data.probabilities, width, color='r')

        x_poss = np.arange(np.deg2rad(30)/2, 2*np.pi, np.deg2rad(30)).tolist()


        bars = self.axes.bar(
            x_poss, [data.probabilities[i] for i in data.plot_ordering],
            width=np.deg2rad(30)-0.01,
            color="#f39c12" if not data.on_dead_rck else "#0000ff",
            edgecolor="black")
 
        self.axes.set_title("Radial Player Tracking cam/laser")
        self.axes.set_ylim([0,1])

      
        bars = self.axes.bar(
            np.pi/2, 1,
            width=np.pi/2,
            color="#FF0000",
            edgecolor="black",
            alpha= .1
        )
        
        front_leg = mlines.Line2D([], [], color="#FF0000", marker='s', alpha= .1,
                                  markersize=15, label='robot front')
        
        occ_leg = mlines.Line2D([], [], color="#f39c12", marker='s',
                                  markersize=15, label='player prob')
        
        drck_leg = mlines.Line2D([], [], color="#0000ff", marker='s',
                                  markersize=15, label='player prob. on dead_rck')

        self.axes.legend(handles=[front_leg, occ_leg, drck_leg], bbox_to_anchor=(1.1, -.001))
       
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()
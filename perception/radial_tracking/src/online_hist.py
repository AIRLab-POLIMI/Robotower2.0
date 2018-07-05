#!/usr/bin/env python
from plotwindow import PlotWindow


import rospy
import sys, random
from PyQt4.QtCore import *
from PyQt4.QtGui import *

import numpy
import matplotlib.mlab as mlab
from scipy.stats import norm
from collections import deque
from std_msgs.msg import Int8
from sensor_msgs.msg import Range
from radial_tracking.msg import SectorProbabilities

class OnlineHist(PlotWindow):
  def __init__(self):
    PlotWindow.__init__(self)

    self.window_size=1000
    self.values= deque(maxlen=self.window_size)  #numpy.zeros((self.window_size))
    self.index=0
    self.draw_counter =0
    self.paused = False

    rospy.init_node('visualizer', anonymous=True)
    self.subscriber = rospy.Subscriber("sector_probabilities", SectorProbabilities, self.plotResults, queue_size = 1 )
    
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
        #n, bins, patches = self.axes.hist(list(self.values), bins = 100, normed=True, facecolor='green', alpha=0.75, align='left')

        #I want to also fit the data to a Gaussian
        # best fit of data
        #(mu, sigma) = norm.fit(list(self.values))
        # add a 'best fit' line
        #y = mlab.normpdf( bins, mu, sigma)
        #l = self.axes.plot(bins, y, 'r--', linewidth=2)

        width = 0.35       # the width of the bars
        bins = self.axes.bar(range(len(data.probabilities)), data.probabilities, width, color='r')

        self.axes.set_title("Particle Filter")
        self.axes.set_xlabel("Sectors")
        self.axes.set_ylabel("Probability")
        self.axes.set_xticks(range(len(data.labels)),data.labels)
        self.axes.set_ylim([0,1])
        # #output= "Data Size: "+str(len(self.values))
        # min_x, max_x=self.axes.get_xlim()
        # min_y, max_y=self.axes.get_ylim()
        #max_x*0.5,max_y*0.5,output,horizontalalignment='left',verticalalignment='center')        
        #self.axes.annotate(output, (0.05,0.9), xycoords = 'axes fraction') 
        self.canvas.draw()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = OnlineHist()
    window.show()
    app.exec_()
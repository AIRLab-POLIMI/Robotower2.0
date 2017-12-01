#!/usr/bin/env python
import rospy
import sys

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtMultimedia import *

import heartbeat
from heartbeat.msg import State
import std_msgs.msg
from arduino_publisher.msg import ImuState
from rosgraph_msgs.msg import TopicStatistics

Node_state = {0:"stopped", 1:"starting", 2:"started"}

# Defining colors
redPalette = QPalette()
redPalette.setColor(QPalette.Foreground,Qt.red)
bluePalette = QPalette()
bluePalette.setColor(QPalette.Foreground,Qt.blue)
greenPalette = QPalette()
greenPalette.setColor(QPalette.Foreground,Qt.green)

Node_state_color = {0: redPalette,
					1: bluePalette,
					2: greenPalette}

class MainWindow(QMainWindow):
	def __init__(self):
		QMainWindow.__init__(self)
		self.setWindowTitle('ROBOGAME monitor')
		cWidget = QWidget(self)
		vbox = QVBoxLayout(cWidget)
	 
		loggerBox = QGroupBox(self)
		loggerBox.setTitle("ROS NODES")
		loggerBox.setStyleSheet("""QGroupBox {
						border: 1px solid gray;
						border-radius: 9px;
						margin-top: 0.5em;
						}
				QGroupBox::title {
							subcontrol-origin: margin;
						left: 10px;
							padding: 0 3px 0 3px;
							}""")
		loggerGrid = QGridLayout(loggerBox) 
		loggerGrid.setHorizontalSpacing(10)
		loggerGrid.setVerticalSpacing(10)

		loggerBox.setLayout(loggerGrid)
		
		vbox.addWidget(loggerBox)

		# Nodes
		arduino_publisher_label = QLabel("arduino_publisher", cWidget)
		procomp_label = QLabel("activity_classifier", cWidget)

		# Topics
		arduino_publisher_imu_topic = QLabel("/imu_state", cWidget)
		arduino_publisher_tower_topic = QLabel("/tower_state", cWidget)
		procomp_topic = QLabel("/player_act_prediction", cWidget)

		# Screen labels
		node_name = QLabel("Node", cWidget)
		node_state = QLabel("Node state", cWidget)
		topic_name = QLabel("Topic name", cWidget)
		topic_state = QLabel("Topic logger state", cWidget)
		topic_window_start = QLabel("window start", cWidget)
		topic_window_stop = QLabel("window stop", cWidget)
		topic_delivered_msgs = QLabel("Delivered msg", cWidget)
		topic_dropped_msgs = QLabel("Dropped msg", cWidget)
		topic_traffic_msgs = QLabel("Traffic (bytes)", cWidget)	

		# Screen labels setAlignment.
		arduino_publisher_label.setAlignment(Qt.AlignCenter) 
		procomp_label.setAlignment(Qt.AlignCenter) 

		# Topics labels setAlignment.
		arduino_publisher_imu_topic.setAlignment(Qt.AlignCenter)
		arduino_publisher_tower_topic.setAlignment(Qt.AlignCenter)
		procomp_topic.setAlignment(Qt.AlignCenter) 
		
		# Statistics setAlignment
		node_name.setAlignment(Qt.AlignCenter)
		node_state.setAlignment(Qt.AlignCenter)
		topic_name.setAlignment(Qt.AlignCenter) 
		topic_state.setAlignment(Qt.AlignCenter)
		topic_window_start.setAlignment(Qt.AlignCenter)
		topic_window_stop.setAlignment(Qt.AlignCenter)
		topic_delivered_msgs.setAlignment(Qt.AlignCenter) 
		topic_dropped_msgs.setAlignment(Qt.AlignCenter) 
		topic_traffic_msgs.setAlignment(Qt.AlignCenter)	      

		# Node state variable
		self.arduino_publisher_state_val = 0
		self.procomp_state_val = 0
		self.prosilica1_state_val = 0 

		# topic logger state variables
		self.arduino_publisher_log_state_val = 0
		self.arduino_publisher_tower_log_state_val = 0
		self.procomp_log_state_val = 0

		# statistic variables
		self.arduino_publisher_dlv_val = 0
		self.arduino_publisher_drp_val = 0
		self.arduino_publisher_bytes_val = 0
		self.arduino_publisher_start_val = 0
		self.arduino_publisher_stop_val = 0
		
		self.arduino_publisher_tower_dlv_val = 0
		self.arduino_publisher_tower_drp_val = 0
		self.arduino_publisher_tower_bytes_val = 0
		self.arduino_publisher_tower_start_val = 0
		self.arduino_publisher_tower_stop_val = 0
		
		self.procomp_dlv_val = 0
		self.procomp_drp_val = 0
		self.procomp_bytes_val = 0
		self.procomp_start_val = 0
		self.procomp_stop_val = 0

		# Init state variables
		self.arduino_publisher_state = QLabel(Node_state[self.arduino_publisher_state_val], cWidget)
		self.procomp_state = QLabel(Node_state[self.procomp_state_val], cWidget)
		self.prosilica1_state = QLabel(Node_state[self.prosilica1_state_val], cWidget) 

		# Init logger state variables
		self.arduino_publisher_log_state = QLabel(Node_state[self.arduino_publisher_log_state_val], cWidget)
		self.arduino_publisher_tower_log_state = QLabel(Node_state[self.arduino_publisher_tower_log_state_val], cWidget)
		self.procomp_log_state = QLabel(Node_state[self.procomp_log_state_val], cWidget)

		# Init stats variables 
		self.arduino_publisher_dlv = QLabel(str(self.arduino_publisher_dlv_val), cWidget)
		self.arduino_publisher_drp = QLabel(str(self.arduino_publisher_drp_val), cWidget)
		self.arduino_publisher_bytes = QLabel(str(self.arduino_publisher_bytes_val), cWidget)
		self.arduino_publisher_start = QLabel(str(self.arduino_publisher_start_val), cWidget)
		self.arduino_publisher_stop = QLabel(str(self.arduino_publisher_stop_val), cWidget)

		self.arduino_publisher_tower_dlv = QLabel(str(self.arduino_publisher_tower_dlv_val), cWidget)
		self.arduino_publisher_tower_drp = QLabel(str(self.arduino_publisher_tower_drp_val), cWidget)
		self.arduino_publisher_tower_bytes = QLabel(str(self.arduino_publisher_tower_bytes_val), cWidget)
		self.arduino_publisher_tower_start = QLabel(str(self.arduino_publisher_tower_start_val), cWidget)
		self.arduino_publisher_tower_stop = QLabel(str(self.arduino_publisher_tower_stop_val), cWidget)

		self.procomp_dlv = QLabel(str(self.procomp_dlv_val), cWidget)
		self.procomp_drp = QLabel(str(self.procomp_drp_val), cWidget)
		self.procomp_bytes = QLabel(str(self.procomp_bytes_val), cWidget)
		self.procomp_start = QLabel(str(self.procomp_start_val), cWidget)
		self.procomp_stop = QLabel(str(self.procomp_stop_val), cWidget)

		# state variable setAlignment
		self.arduino_publisher_state.setAlignment(Qt.AlignCenter)
		self.procomp_state.setAlignment(Qt.AlignCenter) 

		# logger variable setAlignment
		self.arduino_publisher_log_state.setAlignment(Qt.AlignCenter)
		self.arduino_publisher_tower_log_state.setAlignment(Qt.AlignCenter) 
		self.procomp_log_state.setAlignment(Qt.AlignCenter) 

		# stats variable setAlignment
		self.arduino_publisher_dlv.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_drp.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_bytes.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_start.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_stop.setAlignment(Qt.AlignCenter)

		self.arduino_publisher_tower_dlv.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_tower_drp.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_tower_bytes.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_tower_start.setAlignment(Qt.AlignCenter) 
		self.arduino_publisher_tower_stop.setAlignment(Qt.AlignCenter) 
		
		self.procomp_dlv.setAlignment(Qt.AlignCenter) 
		self.procomp_drp.setAlignment(Qt.AlignCenter) 
		self.procomp_bytes.setAlignment(Qt.AlignCenter) 
		self.procomp_start.setAlignment(Qt.AlignCenter) 
		self.procomp_stop.setAlignment(Qt.AlignCenter)
		
		# add widgets
		loggerGrid.addWidget(node_name, 0, 0)
		loggerGrid.addWidget(node_state, 1, 0)
		loggerGrid.addWidget(topic_name, 2, 0)
		loggerGrid.addWidget(topic_state, 3, 0)
		loggerGrid.addWidget(topic_window_start, 4, 0)
		loggerGrid.addWidget(topic_window_stop, 5, 0)
		loggerGrid.addWidget(topic_delivered_msgs, 6, 0) 
		loggerGrid.addWidget(topic_dropped_msgs, 7, 0) 
		loggerGrid.addWidget(topic_traffic_msgs, 8, 0)	
		
		loggerGrid.addWidget(arduino_publisher_label, 0, 1, 1,2) 
		loggerGrid.addWidget(self.arduino_publisher_state, 1, 1, 1, 2)
		loggerGrid.addWidget(arduino_publisher_imu_topic, 2, 1)
		loggerGrid.addWidget(self.arduino_publisher_log_state, 3, 1)
		loggerGrid.addWidget(self.arduino_publisher_start, 4, 1)
		loggerGrid.addWidget(self.arduino_publisher_stop, 5, 1)
		loggerGrid.addWidget(self.arduino_publisher_dlv, 6, 1)
		loggerGrid.addWidget(self.arduino_publisher_drp, 7, 1)
		loggerGrid.addWidget(self.arduino_publisher_bytes, 8, 1)

		loggerGrid.addWidget(arduino_publisher_tower_topic, 2, 2)
		loggerGrid.addWidget(self.arduino_publisher_tower_log_state, 3, 2)
		loggerGrid.addWidget(self.arduino_publisher_tower_start,4,2)
		loggerGrid.addWidget(self.arduino_publisher_tower_stop,5,2)
		loggerGrid.addWidget(self.arduino_publisher_tower_dlv,6,2)
		loggerGrid.addWidget(self.arduino_publisher_tower_drp,7,2)
		loggerGrid.addWidget(self.arduino_publisher_tower_bytes,8,2)
		
		loggerGrid.addWidget(procomp_label, 0, 3) 
		loggerGrid.addWidget(self.procomp_state, 1, 3)
		loggerGrid.addWidget(procomp_topic, 2, 3)  
		loggerGrid.addWidget(self.procomp_log_state, 3, 3)
		loggerGrid.addWidget(self.procomp_start, 4, 3)
		loggerGrid.addWidget( self.procomp_stop, 5, 3)
		loggerGrid.addWidget(self.procomp_dlv, 6, 3)
		loggerGrid.addWidget( self.procomp_drp, 7, 3)
		loggerGrid.addWidget( self.procomp_bytes, 8, 3)
			
		cWidget.setLayout(vbox)
		self.setCentralWidget(cWidget)

	def update_GUI(self):
		self.arduino_publisher_state.setText(Node_state[self.arduino_publisher_state_val])
		self.arduino_publisher_state.setPalette(Node_state_color[self.arduino_publisher_state_val])
		self.procomp_state.setText(Node_state[self.procomp_state_val])
		self.procomp_state.setPalette(Node_state_color[self.procomp_state_val])

		self.procomp_log_state.setText(Node_state[self.procomp_log_state_val])
		self.arduino_publisher_log_state.setText(Node_state[self.arduino_publisher_log_state_val])

		self.arduino_publisher_dlv.setText(str(self.arduino_publisher_dlv_val))
		self.arduino_publisher_drp.setText(str(self.arduino_publisher_drp_val))
		self.arduino_publisher_bytes.setText(str(self.arduino_publisher_bytes_val))
		self.arduino_publisher_start.setText(str(self.arduino_publisher_start_val))
		self.arduino_publisher_stop.setText(str(self.arduino_publisher_stop_val))
		self.procomp_dlv.setText(str(self.procomp_dlv_val))
		self.procomp_drp.setText(str(self.procomp_drp_val))
		self.procomp_bytes.setText(str(self.procomp_bytes_val))
		self.procomp_start.setText(str(self.procomp_start_val))
		self.procomp_stop.setText(str(self.procomp_stop_val))
		

def state_update(data):	
	global main
	if (data.node_name == '/arduino_publisher'):
		main.arduino_publisher_state_val = data.value
	elif (data.node_name == '/activity_classifier'):
		main.procomp_state_val = data.value
	elif (data.node_name == '/arduino/tower_state'):
		main.prosilica1_state_val = data.value
	main.update_GUI()	

def topic_statistics(data):
	global main
	if (data.topic == '/arduino/imu_state'):
		main.arduino_publisher_dlv_val = data.delivered_msgs
		main.arduino_publisher_drp_val = data.dropped_msgs
		main.arduino_publisher_bytes_val = data.traffic
		main.arduino_publisher_start_val = data.window_start.to_sec()
		main.arduino_publisher_stop_val = data.window_stop.to_sec()
	elif (data.topic == '/arduino/tower_state'):
		main.procomp_dlv_val = data.delivered_msgs
		main.procomp_drp_val = data.dropped_msgs
		main.procomp_bytes_val = data.traffic
		main.procomp_start_val = data.window_start.to_sec()
		main.procomp_stop_val = data.window_stop.to_sec()
	
	main.update_GUI()	

def Draw_GUI():
	global main
	app = QApplication(sys.argv)
	main = MainWindow()
	main.show()
	rospy.init_node('GUI', anonymous=True)
 	rospy.Subscriber("state", State, state_update)
	rospy.Subscriber("statistics", TopicStatistics, topic_statistics)
	sys.exit(app.exec_())
	rospy.spin()
    

if __name__ == '__main__':
	try:
	    Draw_GUI()
	except rospy.ROSInterruptException: pass

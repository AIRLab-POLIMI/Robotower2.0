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
from arduino_publisher.msg import TowerState
from rosgraph_msgs.msg import TopicStatistics

Node_state = {0:"FALSE", 1:"TRUE"}

# Defining colors
redPalette = QPalette()
redPalette.setColor(QPalette.Foreground,Qt.red)
bluePalette = QPalette()
bluePalette.setColor(QPalette.Foreground,Qt.blue)
greenPalette = QPalette()
greenPalette.setColor(QPalette.Foreground,Qt.green)

Node_state_color = {0: redPalette,
					1: greenPalette,
					2: bluePalette}

style = """QGroupBox {
						border: 1px solid gray;
						border-radius: 9px;
						margin-top: 0.5em;
						}
				QGroupBox::title {
							subcontrol-origin: margin;
						left: 10px;
							padding: 0 3px 0 3px;
							}"""

class MainWindow(QMainWindow):
	def __init__(self):
		QMainWindow.__init__(self)
		self.setWindowTitle('ROBOGAME monitor')
		cWidget = QWidget(self)
		vbox = QVBoxLayout()#cWidget)
	 
		firstLoggerBox = QGroupBox(self)
		firstLoggerBox.setTitle("1st TOWER")
		firstLoggerBox.setStyleSheet(style)
		firstLoggerGrid = QGridLayout(firstLoggerBox) 
		firstLoggerGrid.setHorizontalSpacing(10)
		firstLoggerGrid.setVerticalSpacing(10)

		secondLoggerBox = QGroupBox(self)
		secondLoggerBox.setTitle("2nd TOWER")
		secondLoggerBox.setStyleSheet(style)
		secondLoggerGrid = QGridLayout(secondLoggerBox) 
		secondLoggerGrid.setHorizontalSpacing(10)
		secondLoggerGrid.setVerticalSpacing(10)

		thirdLoggerBox = QGroupBox(self)
		thirdLoggerBox.setTitle("3rd TOWER")
		thirdLoggerBox.setStyleSheet(style)
		thirdLoggerGrid = QGridLayout(thirdLoggerBox) 
		thirdLoggerGrid.setHorizontalSpacing(10)
		thirdLoggerGrid.setVerticalSpacing(10)

		fourthLoggerBox = QGroupBox(self)
		fourthLoggerBox.setTitle("4th TOWER")
		fourthLoggerBox.setStyleSheet(style)
		fourthLoggerGrid = QGridLayout(fourthLoggerBox) 
		fourthLoggerGrid.setHorizontalSpacing(10)
		fourthLoggerGrid.setVerticalSpacing(10)

		firstLoggerBox.setLayout(firstLoggerGrid)
		secondLoggerBox.setLayout(secondLoggerGrid)
		thirdLoggerBox.setLayout(thirdLoggerGrid)
		fourthLoggerBox.setLayout(fourthLoggerGrid)
		
		vbox.addWidget(firstLoggerBox)
		vbox.addWidget(secondLoggerBox)
		vbox.addWidget(thirdLoggerBox)
		vbox.addWidget(fourthLoggerBox)
	

		# LEDS
		pixmap_red = QPixmap("/home/ewerlopes/catkin_ws/src/logger/scripts/circle_red.png")
		pixmap_red = pixmap_red.scaled(48, 48,Qt.KeepAspectRatio)
		pixmap_green = QPixmap("/home/ewerlopes/catkin_ws/src/logger/scripts/circle_green.png")
		pixmap_green = pixmap_green.scaled(48, 48,Qt.KeepAspectRatio)
		self.Led_state = {0: pixmap_red, 1: pixmap_green}

		# Nodes
		self.tw1_leds = [QLabel(),QLabel(),QLabel(),QLabel()]
		self.tw2_leds = [QLabel(),QLabel(),QLabel(),QLabel()]
		self.tw3_leds = [QLabel(),QLabel(),QLabel(),QLabel()]
		self.tw4_leds = [QLabel(),QLabel(),QLabel(),QLabel()]

		for l1, l2,l3,l4 in zip(self.tw1_leds,self.tw1_leds,self.tw1_leds,self.tw1_leds):
			l1.setPixmap(pixmap_red)
			l1.setAlignment(Qt.AlignCenter)
			l2.setPixmap(pixmap_red)
			l2.setAlignment(Qt.AlignCenter)
			l3.setPixmap(pixmap_red)
			l3.setAlignment(Qt.AlignCenter)
			l4.setPixmap(pixmap_red)
			l4.setAlignment(Qt.AlignCenter)

		# Screen labels
		fst_tower_state = QLabel("Captured?", cWidget)
		fst_led_status = QLabel("LED state:", cWidget)

		sec_tower_state = QLabel("Captured?", cWidget)
		sec_led_status = QLabel("LED state:", cWidget)

		thd_tower_state = QLabel("Captured?", cWidget)
		thd_led_status = QLabel("LED state:", cWidget)

		fth_tower_state = QLabel("Captured?", cWidget)
		fth_led_status = QLabel("LED state:", cWidget)


		# Statistics setAlignment
		fst_tower_state.setAlignment(Qt.AlignCenter)
		sec_tower_state.setAlignment(Qt.AlignCenter)
		thd_tower_state.setAlignment(Qt.AlignCenter)
		fth_tower_state.setAlignment(Qt.AlignCenter)
		fst_led_status.setAlignment(Qt.AlignCenter)
		sec_led_status.setAlignment(Qt.AlignCenter)
		thd_led_status.setAlignment(Qt.AlignCenter)
		fth_led_status.setAlignment(Qt.AlignCenter)

		# Tower status variable
		self.tw1_leds_vals = [0,0,0,0]
		self.tw2_leds_vals = [0,0,0,0]
		self.tw3_leds_vals = [0,0,0,0]
		self.tw4_leds_vals = [0,0,0,0]
		
		self.tw1_status_val = 0
		self.tw2_status_val = 0
		self.tw3_status_val = 0
		self.tw4_status_val = 0

		# Init state variables
		self.tw1 = QLabel(Node_state[0], cWidget)
		self.tw2 = QLabel(Node_state[0], cWidget)
		self.tw3 = QLabel(Node_state[0], cWidget)
		self.tw4 = QLabel(Node_state[0], cWidget)

		# state variable setAlignment
		self.tw1.setAlignment(Qt.AlignCenter)
		self.tw2.setAlignment(Qt.AlignCenter)
		self.tw3.setAlignment(Qt.AlignCenter)
		self.tw4.setAlignment(Qt.AlignCenter)
		
		
		# add widgets
		firstLoggerGrid.addWidget(fst_tower_state, 0, 0)
		firstLoggerGrid.addWidget(fst_led_status, 1, 0)
		secondLoggerGrid.addWidget(sec_tower_state, 0, 0)
		secondLoggerGrid.addWidget(sec_led_status, 1, 0)
		thirdLoggerGrid.addWidget(thd_tower_state, 0, 0)
		thirdLoggerGrid.addWidget(thd_led_status, 1, 0)
		fourthLoggerGrid.addWidget(fth_tower_state, 0, 0)
		fourthLoggerGrid.addWidget(fth_led_status, 1, 0)
		
		firstLoggerGrid.addWidget(self.tw1, 0, 1, 1, 4)
		firstLoggerGrid.addWidget(self.tw1_leds[0], 1, 1)
		firstLoggerGrid.addWidget(self.tw1_leds[1], 1, 2)
		firstLoggerGrid.addWidget(self.tw1_leds[2], 1, 3)
		firstLoggerGrid.addWidget(self.tw1_leds[3], 1, 4)

		secondLoggerGrid.addWidget(self.tw2, 0, 1, 1, 4)
		secondLoggerGrid.addWidget(self.tw2_leds[0], 1, 1)
		secondLoggerGrid.addWidget(self.tw2_leds[1], 1, 2)
		secondLoggerGrid.addWidget(self.tw2_leds[2], 1, 3)
		secondLoggerGrid.addWidget(self.tw2_leds[3], 1, 4)

		thirdLoggerGrid.addWidget(self.tw3, 0, 1, 1, 4)
		thirdLoggerGrid.addWidget(self.tw3_leds[0], 1, 1)
		thirdLoggerGrid.addWidget(self.tw3_leds[1], 1, 2)
		thirdLoggerGrid.addWidget(self.tw3_leds[2], 1, 3)
		thirdLoggerGrid.addWidget(self.tw3_leds[3], 1, 4)

		fourthLoggerGrid.addWidget(self.tw4, 0, 1, 1, 4)
		fourthLoggerGrid.addWidget(self.tw4_leds[0], 1, 1)
		fourthLoggerGrid.addWidget(self.tw4_leds[1], 1, 2)
		fourthLoggerGrid.addWidget(self.tw4_leds[2], 1, 3)
		fourthLoggerGrid.addWidget(self.tw4_leds[3], 1, 4)

			
		cWidget.setLayout(vbox)
		self.setCentralWidget(cWidget)
		self.update_GUI()

	def update_GUI(self):
		self.tw1.setText(Node_state[self.tw1_status_val])
		self.tw1.setPalette(Node_state_color[self.tw1_status_val])
		self.tw2.setText(Node_state[self.tw2_status_val])
		self.tw2.setPalette(Node_state_color[self.tw2_status_val])
		self.tw3.setText(Node_state[self.tw3_status_val])
		self.tw3.setPalette(Node_state_color[self.tw3_status_val])
		self.tw4.setText(Node_state[self.tw4_status_val])
		self.tw4.setPalette(Node_state_color[self.tw4_status_val])
		
		for i in range(4):
			self.tw1_leds[i].setPixmap(self.Led_state[self.tw1_leds_vals[i]])
			self.tw2_leds[i].setPixmap(self.Led_state[self.tw2_leds_vals[i]])
			self.tw3_leds[i].setPixmap(self.Led_state[self.tw3_leds_vals[i]])
			self.tw4_leds[i].setPixmap(self.Led_state[self.tw4_leds_vals[i]])
		

def state_update(data):	
	global main
	if (data.pipe_id == 1):
		main.tw1_leds_vals = data.leds
		main.tw1_status_val = data.is_captured
	elif (data.pipe_id == 2):
		main.tw2_leds_vals = data.leds
		main.tw2_status_val = data.is_captured
	elif (data.pipe_id == 3):
		main.tw3_leds_vals = data.leds
		main.tw3_status_val = data.is_captured
	elif (data.pipe_id == 4):
		main.tw4_leds_vals = data.leds
		main.tw4_status_val = data.is_captured

	main.update_GUI()	

def Draw_GUI():
	global main
	app = QApplication(sys.argv)
	main = MainWindow()
	main.show()
	rospy.init_node('GUI', anonymous=True)
 	rospy.Subscriber("/arduino/tower_state", TowerState, state_update)
	sys.exit(app.exec_())
	rospy.spin()
    

if __name__ == '__main__':
	try:
	    Draw_GUI()
	except rospy.ROSInterruptException: pass

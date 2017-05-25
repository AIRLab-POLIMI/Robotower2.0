PhD Robogame ROS stack
======================
![ROS](https://img.shields.io/badge/ROS-Indigo-brightgreen.svg) ![ROS](https://img.shields.io/badge/ROS-Kinetic-brightgreen.svg) ![OS](https://img.shields.io/badge/OS-Ubuntu%2016.04-orange.svg)

This is the repository for the robogame project during my PhD studies at Politecnico di Milano, Italy.

**Game related repositories:**<br/>
* `robogame` -- metapackage. <br/>
* `arduino_sketches` -- contains arduino programs used on game towers, player imu sensor (transmitter/receiver).  <br/>
* `kinect_tracker` -- detects and publisher player position.  <br/>
* `tower_manager` -- a node to publish useful game tower data. <br/>

**Robot base repositories:**</br>
* `triskarone` -- interprets velocity comands for moving the base.</br>
  * `r2p`-- contains msgs used by the r2p (first version) boards</br>
* `restamp_encoders` -- subscribe to encoder messages and re-stamp them. The original encoder topics are not stamped. <br/>
* `triskar_kinematics` -- subscribe to stamped encoder topics and publishes the robot velocity.<br/>


**Supporting repositories:**</br>

* `heartbeat` -- package to monitor nodes status. <br/>


* `joystick` -- the joystick node for controlling the robot with a PS3-like joystick. <br/>
* `record_bag` - a package to record bag files by pressing a joystick button.
* `video_recorder` -- for publishing image frames from a external USB camera.<br/>

**Additional folders (NOT REPOS):**</br>

Inside auxiliary folder, we have:

* `_gnuplot` -- folder with some predefined config files for using gnuplot.</br>
* `_arduino` -- folder containing arduino sketches used on onboard arduino and the ones located on the game towers.</br>
* `_help` -- folder containing some infos regarding setup issues.</br>
* `_scripts` -- folder containing some useful scripts and programs.</br>

Dependencies
============

* OpenCV
* LibFreenect2
* IAI Kinect2

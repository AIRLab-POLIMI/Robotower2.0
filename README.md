PhD Robogame ROS stack
======================
![ROS](https://img.shields.io/badge/ROS-Indigo-brightgreen.svg)
![OS](https://img.shields.io/badge/OS-Ubutu14.04-orange.svg)

This is the repository for the robogame project during my PhD studies at Politecnico di Milano, Italy.

**Game related repositories:**<br/>
* `robogame` -- metapackage. <br/>
* `robogame_arduino` -- The robogame_arduino package defines msgs that the arduino module attached to the robot is going to use.  <br/>
* `robogame_game_manager` -- where actions and conditions (bt leaves) codes are defined. <br/>
* `robogame_imu_sensor` -- packages that pusblish data received from the MPU6050 (acceleromenter) attached to the player. <br/>
* `robogame_kinectfeatures_extractor` -- for extracting and publishing player features using microsoft kinect 2.  <br/>
* `robogame_proximity_sensor` -- for publishing robot ultrasound sensor data. <br/>

**Robot base repositories:**</br>
* `triskarone` -- interprets velocity comands for moving the base.</br>
* `r2p`	-- contains msgs used by the r2p (first version) boards</br>
* `restamp_encoders` -- subscribe to encoder messages and re-stamp them. The original encoder topics are not stamped. <br/>
* `triskar_kinematics` -- subscribe to stamped encoder topics and publishes the robot velocity.<br/>


**Supporting repositories:**</br>
* `rosserial` --  used for allowing ros-arduino communication.</br>
* `robogame_joystick` -- for, of course, controlling the robot with a PS3-like joystick. <br/>
* `video_recorder` -- for publishing image frames from a camera.<br/>

**Additional folders (NOT REPOS):**</br>
* `_gnuplot` -- folder with some predefined config files for using gnuplot.</br>
* `_arduino` -- folder containing arduino sketches used on onboard arduino and the ones located on the game towers.</br>
* `_help` -- folder containing some infos regarding setup issues.</br>
* `_scripts` -- folder containing some useful scripts and programs.</br>

Dependencies
============

* OpenCV
* LibFreenect2

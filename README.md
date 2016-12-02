PhD Robogame ROS stack
======================
![ROS](https://img.shields.io/badge/ROS-Indigo-brightgreen.svg)
![OS](https://img.shields.io/badge/OS-Ubutu14.04-orange.svg)

This is the repository for the robogame project during my PhD studies at Politecnico di Milano, Italy.

**Game related repositories:**<br/>
* `robogame` -- metapackage. <br/>
* `robogame_arduino` --  <br/>
* `robogame_game_manager` -- where actions and conditions (bt leaves) codes are defined. <br/>
* `robogame_imu_sensor` -- packages that pusblish data received from the MPU6050 (acceleromenter) attached to the player. <br/>
* `robogame_joystick` -- for, of course, controlling the robot with a PS3-like joystick. <br/>
* `robogame_kinectfeatures_extractor` -- for extracting and publishing player features using microsoft kinect 2.  <br/>
* `robogame_proximity_sensor` -- for publishing robot ultrasound sensor data. <br/>
* `robogame_video_recorder` -- for publishing image frames from a camera.<br/>

**Robot base repositories:**</br>
* `triskarone` -- interprets velocity comands for moving the base.</br>
* `r2p`	-- contains msgs used by the r2p (first version) boards</br>

**Supporting repositories:**</br>
* `rosserial` --  used for allowing ros-arduino communication.</br>

**Additional folders (NOT REPOS):**</br>
* `_gnuplot` -- folder with some predefined config files for using gnuplot.</br>
* `_arduino` -- folder containing arduino sketches used on onboard arduino boards.</br>
* `_help` -- folder containing some infos regarding setup issues.</br>

Dependencies
============

* OpenCV
* LibFreenect2

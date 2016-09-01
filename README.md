PhD Robogame ROS stack
======================
![ROS](https://img.shields.io/badge/ROS-Indigo-brightgreen.svg)
![OS](https://img.shields.io/badge/OS-Ubutu14.04-orange.svg)

This is the repository for the robogame project during my PhD studies at Politecnico di Milano, Italy.

* `robogame` -- metapackage. <br/>
* `robogame_bt_core` -- where the behavior tree (bt) main code resides. <br/>
* `robogame_bt_leaves` -- where actions and conditions (bt leaves) codes are defined. <br/>
* `robogame_simulator` -- has codes for bringing up the gazebo simulator. <br/>

Dependencies
============

* [Turtlebot ROS packages](http://wiki.ros.org/turtlebot_gazebo) <br/>

Setup
=====
Git clone the repository into your catkin workspace: <br/>
`$ cd path/to/catkin/repository` <br/>
`$ cd src` <br/>
`$ git clone https://github.com/ewerlopes/phd_robogame.git` <br/>

Build: <br/>
`$ cd ..` <br/>
`$ catkin_make` <br/>


How to use it
=============
**Bring up**  <br/>
`$ roslaunch robogame_simulator robogame.launch` <br/>

**Navigation** <br/>
`$ roslaunch robogame_simulator nav_demo.launch` <br/>

**For visualization:** <br/>
`$ rosrun image_view image_view image:=/camera/depth/image_raw` <br/>
`$ rosrun image_view image_view image:=/camera/rgb/image_raw` <br/>
`$ roslaunch turtlebot_rviz_launchers view_navigation.launch` <br/>

Additional Info
===============
At this point I am using the `turtlebot` as the target robot base. So, there's a large dependency on `turtlebot` ros packages. This dependency is going to be descontinued soon.

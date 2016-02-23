PhD Robogame ROS stack
======================

This is the repository for the robogame project during my PhD studies at Politecnico di Milano, Italy.

* robogame -- metapackage. <br/>
* robotgame_bt -- game behavior tree. <br/>
* robogame_simulator -- gazebo simulator. <br/>

How to use it
=============

**Bring up**
`$ roslaunch robogame_simulator robogame.launch`

**Navigation**


**For visualization:**
`$ rosrun image_view image_view image:=/camera/depth/image_raw` 
`$ rosrun image_view image_view image:=/camera/rgb/image_raw`
`$ roslaunch turtlebot_rviz_launchers view_navigation.launch`

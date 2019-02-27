# Triskar3
New triskar ros nodes and messages

Installation and setup of AIRLab Triskar-family robots
------------------------------------------------------

This is a description of the installation procedure for Triskar-family robots (Triskarino, Triskarone, and so on) based on the Raspberry Pi and Nova Core modules to manage the three motors of the robots. Except for small differences, this procedure also applies to robots using Nova Core modules not belonging to the Triskar family (e.g., robots using a PC as onboard computer instead of a Raspberry Pi; robots with two instead of three motors).

At the end of this procedure, the robot will be configured as follows:

- ROS available, with roscore running right from startup
- NovaCore modules interfaced with ROS via serial link and rosserial
- Logitech F710 joypad active
- manual driving of the robot using the joypad active right from startup
- no need to access the robot via SSH if manual driving is all you need


Install Linux
-------------
Install Ubuntu Mate on the SD card by following the instructions at
```
https://ubuntu-mate.org/raspberry-pi/
```
Take care of using an SD card with sufficient capacity for Ubuntu and ROS; at least 16 GB are suggested.

When prompted by the Ubuntu Mate installer, define
* a user called "airlab"
* "aerolabio" as the user password
* "<name_of_your_robot>" as the Raspberry Pi machine name

This way, whenever you need to log in to the robot via SSH you will use command
```
ssh airlab@<name_of_your_robot>
```

After installation, open a terminal and install some additional useful Ubuntu packages with command
```
sudo apt install ssh daemontools git
```
Remove Ubuntu package modemmanager (which can interfere with serial communication) with command
```
sudo apt purge modemmanager
```
Finally, add the airlab user to user group *dialout* with command
```
sudo adduser airlab dialout
```
(this enables the user to manage serial devices).


Install ROS
-----------
Install ROS by following the instructions at
```
http://wiki.ros.org/kinetic/Installation/Ubuntu
```
(or corresponding page for ROS versions different from kinetic).

Instead of the Desktop-Full Install, it may be better to perform the Desktop Install procedure, which avoids many ROS packages that are scarcely suitable for a low-processing-power platform such as the Raspberry Pi (e.g., simulation packages).

Install additional ROS packages with the following command:
```
sudo apt install ros-kinetic-rosserial ros-kinetic-joy ros-kinetic-navigation ros-kinetic-teleop-twist-joy
```
(if you are not using ROS kinetic, change "kinetic" into the name of your version).

Then, create the directory to be used as ROS workspace with command
```
mkdir ~/ros
```
(note that this is NOT the standard name, which is catkin_ws; the non-standard name is necessary because of absolute file paths used in some parts of the software) and create subdirectory *src* with command
```
mkdir ~/ros/src
```
Finally, in order to let ROS find your own packages, use *pluma* (the standard text editor of Ubuntu Mate) to open file *~/.bashrc* and add the command
```
source /home/airlab/ros/devel/setup.bash
```
as its last line.



Install additional software from GitHub
---------------------------------------
Install and compile packages needed by the robot with the following commands:
```
cd ~/ros/src
git clone https://github.com/AIRLab-POLIMI/Triskar3.git
git clone https://github.com/boris-il-forte/robot_upstart.git
git clone https://github.com/novalabs/ros-rosserial_core.git
catkin_make
rospack profile
```
(the last is a ROS command that can solve some issues where ROS is not able to find packages).


Prepare the robot
-----------------
To ensure that the serial interface of the Nova Core boards always has the same peripheral name, follow the instructions in
```
~/ros/src/Triskar3/triskar/udev/README.md
```


Set up robot_upstart
--------------------
*robot_upstart* is a system that enables running a ROS system (defined by a launchfile) as a linux system service. Therefore, once set up the ROS system will be managed using the usual linux *service* command. In particular, it is possible to run, stop, restart and get the current status of the ROS system using linux commands
```
sudo service <name_of_service> start
sudo service <name_of_service> stop
sudo service <name_of_service> restart
sudo service <name_of_service> status
``` 
This is especially useful for robots, as the service can be automatically run at startup and runs in background. For Triskar-family robots, the system provides motor control and manual driving via the Logitech F710 joypad.

In order to use robot_upstart, it is necessary to define a ROS launchfile that runs all the components of the system. For a Triskar-family robot, such launchfile is
```
~/ros/Triskar3/triskar/launch/triskar_minimal.launch
```
To set up robot_upstart, use command
```
rosrun robot_upstart install ~/ros/source/Triskar3/triskar/launch/triskar_minimal.launch
```
and then execute the command suggested onscreen. 

This procedure creates a new service called *triskar* which is automatically run as soon as the Raspberry Pi starts up.

Because of their nature, services operate transparently and do not provide feedback to the user. For a ROS system this can occasionally be an annoying feature, especially during debugging. Therefore, robot_upstart provides an alternative way to run the system which corresponds to running the launchfile in a terminal window, and provides the same output. To do so, open a terminal window and run the following commands:
```
sudo service triskar stop
sudo triskar-start
```
The first command is necessary only if the *triskar* service is running, to avoid conflicts. The ROS system running in the terminal window can be stopped with the usual method, i.e. by pressing Ctrl-C within the window (or by closing the window).

Note that robot_upstart provides an uninstallation procedure which (if necessary) can be used to remove the service corresponding to the ROS system.

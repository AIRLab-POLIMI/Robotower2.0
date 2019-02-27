Raspberry Pi scripts
====================

To install the shutdown button script:

sudo vim /etc/rc.local


add before exit 0 the following line:
sudo python /home/airlab/ros/src/Triskar3/scripts/shutdown_button.py & 

Notice that you have to change the above command if you are not using the standard triskar naming convention, i.e. you have to change 

/home/airlab/ros/src/

with the appropriate path to the Triskar3 directory

Udev rules
==========

Udev Rules needed by the triskar
To install udev rules, execute:

sudo cp $(rospack find triskar)/udev/* /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger


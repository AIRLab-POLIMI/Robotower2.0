#
#	Defines the robot battery voltage info measured by the arduino
#

std_msgs/Header header
float32 raw_voltage		# raw analog voltage from the voltage divider circuit
float32 voltage			# processed real voltage level of the battery

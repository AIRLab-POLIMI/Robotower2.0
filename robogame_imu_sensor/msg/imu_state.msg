#-----------------------------
# Header
#----------------------

std_msgs/Header header

#----------------------
# Gyro reading
#-----------------------------
# In degrees/sec.

geometry_msgs/Vector3 gyro

#----------------------
# Accelerometer reading
#-----------------------------
# Acceleration in m/sec^2.

geometry_msgs/Vector3 linear_acc

#----------------------
# Quaternion reading
#-----------------------------
# the quaternion given by the DMP of the MPU6050.

float32[4] q        # [w, x, y, z]         quaternion container

#!/usr/bin/env python
import rospy
import collections
import numpy as np
from activity_utils import *
from arduino_publisher.msg import ImuState
from activity_recognition.msg import PlayerActivityPrediction
from motion_primitive import MotionPrimitive
from heartbeat.msg import State
from heartbeat.HeartbeatClientPython import *


OVERLAP = 5             # sample window overlap
WINDOW_SIZE = 50        # sample windows size
ASSERTION_RETRIES = 1   # number of retries for assertion failure

### windowing function
window_rads = np.linspace(0, np.pi, WINDOW_SIZE)
windowing_func = np.sin(window_rads)**2

# heartbeat client
hb = HeartbeatClientPy()

# overlap buffers
x_buffer_overlap = collections.deque(maxlen=OVERLAP)
y_buffer_overlap = collections.deque(maxlen=OVERLAP)
z_buffer_overlap = collections.deque(maxlen=OVERLAP)
# accelerometer data buffers
acc_x_buffer = collections.deque(maxlen=WINDOW_SIZE-OVERLAP)
acc_y_buffer = collections.deque(maxlen=WINDOW_SIZE-OVERLAP)
acc_z_buffer = collections.deque(maxlen=WINDOW_SIZE-OVERLAP)

# MotionPrimitive object
motion_primitive = None
# time of las message arrival
last_arrive_at = None

# classification target ids.
CLF_TABLE = {1: 'LM', 2:'R', 3: 'WD'}

def callback(data):
    global  acc_x_buffer, acc_y_buffer, acc_z_buffer, last_arrive_at
    # Normalizing the accelerometer data (1 g equals 16384.0) prior to saving it. 
    acc_x_buffer.append(data.linear_acc.x / 16384.0)
    acc_y_buffer.append(data.linear_acc.y / 16384.0)
    acc_z_buffer.append(data.linear_acc.z / 16384.0)
    last_arrive_at = data.header.stamp


def onExit():
	print "shutdown time!"
	hb.set_node_state(State.STOPPED)
	hb.stop('activity_classifier')

def main():

    global hb, motion_primitive, acc_x_buffer, acc_y_buffer, acc_z_buffer, \
            x_buffer_overlap, y_buffer_overlap, z_buffer_overlap, ASSERTION_RETRIES

    # setting signal_shutdown callback
    rospy.on_shutdown(onExit)
    # init ros
    rospy.init_node('activity_classifier', anonymous=False)
    # Subscribe to imu_state
    rospy.Subscriber('/arduino/imu_state', ImuState, callback)

    # Define publisher
    pub = rospy.Publisher('player_act_prediction', PlayerActivityPrediction, queue_size=10)

    # define loope rate
    rate = rospy.Rate(100)
    # define heartbeat client
    hb.start('/activity_classifier')
    # set heartbeat client state
    hb.set_node_state(State.INIT)

    # get init time
    init_time = rospy.get_rostime()
    # define motion_primitive object
    motion_primitive = MotionPrimitive(init_time)
    # set new heartbeat client state
    hb.set_node_state(State.STARTED)
    
    while not rospy.is_shutdown():
        # issue heartbeat
        hb.alive('activity_classifier')

        ## Checking wheather we have achieve WINDOW_SIZE.
        if len(acc_x_buffer) == WINDOW_SIZE-OVERLAP and \
            len(acc_y_buffer) == WINDOW_SIZE-OVERLAP and \
            len(acc_z_buffer) == WINDOW_SIZE-OVERLAP:

            ## Adding new windows with overlap
            ## MULTIPLY BY WINDOWING FUNCTION TO ATTENUATE WINDOWS ENDPOINTS.
            x_windows = []
            y_windows = []
            z_windows = []
            
            try:
                x_windows = running_mean(np.array(list(x_buffer_overlap) + list(acc_x_buffer)) * windowing_func, N=3)
                y_windows = running_mean(np.array(list(y_buffer_overlap) + list(acc_y_buffer)) * windowing_func, N=3)
                z_windows = running_mean(np.array(list(z_buffer_overlap) + list(acc_z_buffer)) * windowing_func, N=3)
            except ValueError as error:
                rospy.logwarn("ValueError: " + str(error) + "\nNOTE: IT'S NORMAL IF IT OCCURS ONLY ONCE!")
            except RuntimeWarning as warn:
                rospy.logwarn(str(warn))

            # control windows size
            try:
                assert(len(x_windows) == WINDOW_SIZE and \
                        len(y_windows) == WINDOW_SIZE and \
                        len(z_windows) == WINDOW_SIZE)
                
                 ## Taking the mean of standard deviation.
                mean = np.mean([np.std(x_windows), 
                                np.std(y_windows), 
                                np.std(z_windows)])
                
                # check wheather motion primitive is defined.
                if not motion_primitive.expand(mean, last_arrive_at):
                    if motion_primitive.current_state == MotionPrimitive.SIGNALS['CLOSED']:
                        mp = motion_primitive.getFeatures()
                        if mp is not None:
                            data = mp["prediction"]
                            duration = mp["duration"]
                            
                            msg = PlayerActivityPrediction()
                            msg.header.stamp = rospy.Time.now()
                            msg.prediction   = data
                            pub.publish(msg)

                            rospy.loginfo("Motion primitive lasted: {} secs".format(duration))

            except AssertionError as error:
                if ASSERTION_RETRIES:
                    ASSERTION_RETRIES -= 1
                    rospy.logwarn("WINDOW_SIZE less than the value set. \
                                    Remaining retries: %d", ASSERTION_RETRIES)
                else:
                    rospy.signal_shutdown("ERROR: windows of data not equal to WINDOW_SIZE!")

            ## Saving overlap
            x_buffer_overlap.extend(list(acc_x_buffer)[-OVERLAP:])
            y_buffer_overlap.extend(list(acc_y_buffer)[-OVERLAP:])
            z_buffer_overlap.extend(list(acc_z_buffer)[-OVERLAP:])

            # ensure overlap buffers have OVERLAP size
            assert(len(x_buffer_overlap) == OVERLAP and \
                len(y_buffer_overlap) == OVERLAP and \
                len(z_buffer_overlap) == OVERLAP)

            ## Clear the buffer
            acc_x_buffer.clear()
            acc_y_buffer.clear()
            acc_z_buffer.clear()
        
        # adjust rate and loop
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

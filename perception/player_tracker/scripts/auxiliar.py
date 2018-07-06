from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import rospy
import copy
import tf

class KinectEvidence:
    '''This class maintains the kinect evidence about where the player is located'''
    def __init__(self, divisions=30):
        self.divisions = divisions
        self.multinomial = np.ones(divisions)/divisions
        print self.multinomial
    
    def update(players):
        for p in players:
            rospy.loginfo("P. index: {}".format(math.atan2(p.point.y,p.point.x)%(np.pi/self.divisions)))

    
    def euler(self,x):
        """Euler's formula"""
        return np.exp(x*1j)

    

class PolarGrid:
    def __init__(self, n_sections=30, fframe='base_link', cframe='base_link'):
        self.n_sections = n_sections
        self.namespace = "polar_occupancy_grid"
        self.lines_pub = rospy.Publisher('polar_markers', Marker, queue_size=1)
        self.arrow_pub = rospy.Publisher('polar_arrow_marker', Marker, queue_size=1)
        self.fov_pub = rospy.Publisher('fov_marker', Marker, queue_size=1)
        self.text_pub = rospy.Publisher('text_marker', Marker, queue_size=1)
        self.line_scale = 4
        self.tf_listener = tf.TransformListener()
        self.fixed_frame = fframe
        self.camera_frame = cframe
        self.cam_width = 512                # camera frame
        self.width_fov = 1.23220245191      # 70.6*M_PI/180  Kinect (polar coodinate)

        self.FOV_low_endpoint = self.get_camera_bearing(0)
        self.FOV_hig_endpoint = self.get_camera_bearing(self.cam_width-1)
    
    def euler(self,x):
        """Euler's formula"""
        return np.exp(x*1j)

    def get_camera_bearing(self,x):
        return (0.5 - x / float(self.cam_width)) * self.width_fov;

    def publish_fov(self, time):
        # publish rviz markers       
        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = time
        marker.ns = self.namespace
        marker.type = Marker.LINE_LIST
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # marker color
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        x, y, _ = self.getRobotPose()
        sec = [self.euler(self.FOV_low_endpoint), self.euler(self.FOV_hig_endpoint)*-1]
        for s in range(len(sec)):
            sec[s] += self.euler(np.pi/2)
            p = Point()
            p.x = x
            p.y = y
            marker.points.append(copy.deepcopy(p))
            p1 = Point()
            p1.x = sec[s].imag * 2.5
            p1.y = sec[s].real * 2.5
            marker.points.append(copy.deepcopy(p1))

        self.fov_pub.publish(marker)
        

    def getRobotPose(self):
        """
        Gets robot global position. That is, performs a TF transformation from /base_link to /map and returns
        x,y and theta.
        OUTPUTS:
        @ a 3D-numpy array defined as: [x, y, theta] w.r.t /map.
        """
        try:
            self.tf_listener.waitForTransform(self.fixed_frame,self.fixed_frame, rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self.tf_listener.lookupTransform(self.fixed_frame,self.fixed_frame, rospy.Time(0))
            # transform from quaternion to euler angles
            euler = tf.transformations.euler_from_quaternion(rot)
 
            return [trans[0], trans[1], euler[2]]   # [xR,yR,theta]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Navigation node: " + str(e))
    
    def publish_marker(self, people, time):
        # publish rviz markers       
        marker = Marker()
        marker.header.frame_id = self.fixed_frame
        marker.header.stamp = time
        marker.ns = self.namespace
        marker.type = Marker.LINE_LIST
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # marker color
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        x, y, _ = self.getRobotPose()
        
        interval = np.linspace(0,2*np.pi, self.n_sections)
        sec = [self.euler(i) for i in interval]
        for i,s in enumerate(range(len(sec))):
            p = Point()
            p.x = x
            p.y = y
            marker.points.append(copy.deepcopy(p))
            p1 = Point()
            p1.x = sec[s].real * self.line_scale
            p1.y = sec[s].imag * self.line_scale
            marker.points.append(copy.deepcopy(p1))

            # Text showing person's ID number
            m = Marker()
            m.header.frame_id = self.fixed_frame
            m.header.stamp = time
            m.ns = self.namespace
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.text = str(i)
            m.scale.z = 0.2
            m.pose.position.x = sec[s].real * self.line_scale
            m.pose.position.y = sec[s].imag * self.line_scale         
            m.pose.position.z = 0.01
            self.text_pub.publish(m)


        # for p in people:
        #     rospy.logwarn("P. index: {}".format(int(math.atan2(p.point.y,p.point.x)/(2*np.pi/30))))
        
        self.lines_pub.publish(marker)
        self.publish_fov(time)

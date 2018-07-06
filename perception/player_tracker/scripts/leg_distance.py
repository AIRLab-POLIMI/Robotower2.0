#!/usr/bin/env python2
import rospy
import tf
import math
import copy
import numpy as np
from scipy import spatial

# Custom messages
from player_tracker.msg import Person, PersonArray, Leg, LegArray, PersonEvidence, PersonEvidenceArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PolygonStamped, Point32, PointStamped

from munkres import Munkres, print_matrix # For the minimum matching assignment problem. To install: https://pypi.python.org/pypi/munkres 
from scipy.stats import beta

from sklearn.externals import joblib

class likelihood:
    '''Implements a Bernoulli distribution'''
    
    def __init__(self, data):
        """Likelihood for binary data."""
        self.counts = {s:0 for s in ['0', '1']}
        self._process_data(data)
 
    def _process_data(self, data):
        """Process data."""
        temp = [str(x) for x in data]
        for s in ['0', '1']:
            self.counts[s] = temp.count(s)

        if len(temp) != sum(self.counts.values()):
            raise Exception("Passed data is not all 0`s and 1`s!")
    
    def _process_probabilities(self, p0):
        """Process probabilities."""
        n0 = self.counts['0']
        n1 = self.counts['1']

        if p0 != 0 and p0 != 1:
            # typical case
            logpr_data = n0*np.log(p0) + n1*np.log(1.-p0)
            pr_data = np.exp(logpr_data)
        elif p0 == 0 and n0 != 0:
            # p0 can't be 0 if n0 is not 0
            logpr_data = -np.inf
            pr_data = np.exp(logpr_data)
        elif p0 == 0 and n0 == 0:
            # data consistent with p0=0
            logpr_data = n1*np.log(1.-p0)
            pr_data = np.exp(logpr_data)            
        elif p0 == 1 and n1 != 0:
            # p0 can't be 1 if n1 is not 0
            logpr_data = -np.inf
            pr_data = np.exp(logpr_data)
        elif p0 == 1 and n1 == 0:
            # data consistent with p0=1
            logpr_data = n0*np.log(p0)
            pr_data = np.exp(logpr_data)

        return pr_data, logpr_data
        
    def prob(self, p0):
        """Get probability of data."""
        pr_data, _ = self._process_probabilities(p0)
        return pr_data
    
    def log_prob(self, p0):
        """Get log of probability of data."""
        _, logpr_data = self._process_probabilities(p0)

        return logpr_data


class prior:
    '''Beta prior for binary data.'''
    
    def __init__(self, alpha0=1, alpha1=1):
        self.a0 = alpha0
        self.a1 = alpha1
        self.p0rv = beta(self.a0, self.a1)

    def interval(self, prob):
        """End points for region of pdf containing `prob` of the
        pdf-- this uses the cdf and inverse.
        
        Ex: interval(0.95)
        """
        return self.p0rv.interval(prob)

    def mean(self):
        """Returns prior mean."""
        return self.p0rv.mean()

    def pdf(self, p0):
        """Probability density at p0."""
        return self.p0rv.pdf(p0)

    def plot(self, ax=''):
        """A plot showing mean and 95% credible interval."""
        fig = ""
        if ax == '':
            fig, ax = plt.subplots(1, 1)
        else:
            x = np.arange(0., 1., 0.01)

            # get prior mean p0
            mean = self.mean()

            # get low/high pts containg 95% probability
            low_p0, high_p0 = self.interval(0.95)
            x_prob = np.arange(low_p0, high_p0, 0.01)

            # plot pdf
            ax.plot(x, self.pdf(x), 'r-')

            # fill 95% region
            ax.fill_between(x_prob, 0, self.pdf(x_prob),
                            color='red', alpha='0.2' )

            # mean
            ax.stem([mean], [self.pdf(mean)], linefmt='r-', 
                    markerfmt='ro', basefmt='w-')

            ax.set_xlabel('Probability of Zero')
            ax.set_ylabel('Prior PDF')
            ax.set_ylim(0., 1.1*np.max(self.pdf(x)))

        if ax == '':
            plt.show()


class posterior:
    '''Implements a Beta-Bernoulli model'''
    def __init__(self, data, prior):
        """The posterior.
        data: a data sample as list
        prior: an instance of the beta prior class
        """
        self.likelihood = likelihood(data)
        self.prior = prior
        self._process_posterior()

    def _process_posterior(self):
        """Process the posterior using passed data and prior."""

        # extract n0, n1, a0, a1 from likelihood and prior
        self.n0 = self.likelihood.counts['0']
        self.n1 = self.likelihood.counts['1']
        self.a0 = self.prior.a0
        self.a1 = self.prior.a1
        self.p0rv = beta(self.a0 + self.n0, self.a1 + self.n1)
    
    def interval(self, prob):
        """End points for region of pdf containing `prob` of the
        pdf.
        Ex: interval(0.95)
        """
        return self.p0rv.interval(prob)

    def mean(self):
        """Returns posterior mean."""
        return self.p0rv.mean()

    def pdf(self, p0):
        """Probability density at p0."""
        return self.p0rv.pdf(p0)

    def plot(self, figSize=(8,3), supertitle='', left=0.12, right=0.97,
                    bottom=0.21, top=.9, wspace=0.5):
        """A plot showing prior, likelihood and posterior."""
        
        fig = plt.figure(figsize=figSize)
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top, wspace=wspace)

        x = np.arange(0., 1., 0.01)
        
        ax0= plt.subplot(131)

        ## Prior
        # get prior mean p0
        pri_mean = self.prior.mean()

        # get low/high pts containg 95% probability
        pri_low_p0, pri_high_p0 = self.prior.interval(0.95)
        pri_x_prob = np.arange(pri_low_p0, pri_high_p0, 0.01)

        # plot pdf
        ax0.plot(x, self.prior.pdf(x), 'r-')

        # fill 95% region
        ax0.fill_between(pri_x_prob, 0, self.prior.pdf(pri_x_prob),
                           color='red', alpha='0.2' )

        # mean
        ax0.stem([pri_mean], [self.prior.pdf(pri_mean)],
                   linefmt='r-', markerfmt='ro',
                   basefmt='w-')

        ax0.set_title('Prior PDF')
        ax0.set_ylabel('$p(x)$')
        ax0.set_ylim(0., 1.1*np.max(self.prior.pdf(x)))

        ax1= plt.subplot(132)
        ## Likelihood
        # plot likelihood
        lik = [self.likelihood.prob(xi) for xi in x]
        ax1.plot(x, lik, 'k-')
        ax1.set_ylabel('$p(x)$')
        ax1.set_title('Likelihood')

        ax2= plt.subplot(133)
        ## Posterior
        # get posterior mean p0
        post_mean = self.mean()

        # get low/high pts containg 95% probability
        post_low_p0, post_high_p0 = self.interval(0.95)
        post_x_prob = np.arange(post_low_p0, post_high_p0, 0.01)

        # plot pdf
        ax2.plot(x, self.pdf(x), 'b-')

        # fill 95% region
        ax2.fill_between(post_x_prob, 0, self.pdf(post_x_prob),
                           color='blue', alpha='0.2' )

        # mean
        ax2.stem([post_mean], [self.pdf(post_mean)],
                   linefmt='b-', markerfmt='bo',
                   basefmt='w-')

        ax2.set_xlabel('Probability of Zero')
        ax2.set_ylabel('$p(x)$')
        ax2.set_title('Posterior PDF')
        ax2.set_ylim(0., 1.1*np.max(self.pdf(x)))

        plt.suptitle(supertitle)
        plt.show()

class LegContextProcessor:
    ''' 
    Process pairs of leg clusters and weight them by their
    probability of being a human using a generative model trained
    by data.
    
    Additionally, use a beta-bernoulli model for controlling the
    combinatorial explosion on the number of leg pair evaluations.
    
    '''
    
    def __init__(self, model_name):
        self.model = joblib.load(model_name)
        self.cutoff = prior(1, 1)                # prior with uniform parameters

    
    def updateCutoff(self, data):
        '''Updates the cutoff probability for the getDistance() method'''
        self.cutoff = posterior(data, self.cutoff)
    
    def getProbability(self, distance):
        '''Evaluate pair distance probability'''
        logprob = self.model.score_samples(np.array(distance))   # get log-probability
        return np.exp(logprob)

class LegDistance: 

    '''Calculates the distance between LEG clusters and saves to file.''' 
    
    def __init__(self, p1, p2, p3, p4, logfile=''):

        # TODO: change this path to be acquire from parameter!
        self.leg_context = LegContextProcessor('/home/airlab/catkin_ws/src/phd_robogame/perception/player_tracker/model/leg_distance-gmm.pkl')

        self.tf_listener = tf.TransformListener()
        self.marker_pub = rospy.Publisher('detected_legs_in_bounding_box', Marker, queue_size=300)
        self.evidence_pub = rospy.Publisher('evidence_of_player', PersonEvidenceArray, queue_size=3)

        self.isSaveToFile = logfile != ''
        self.fixed_frame = rospy.get_param('fixed_frame')

        if self.isSaveToFile:
            self.f = open(logfile, "w")
        

        # accepted area
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4

        self.bound_box_points = []
        for p in [self.p1, self.p2, self.p3, self.p4]:
            self.bound_box_points.append((p.x,p.y))

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', PolygonStamped, queue_size=1)

        #rospy.spin() # So the node doesn't immediately shut down

    
    def publish(self):
        '''Publish poligon for the bouding box'''
            
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'map'
        polygon.polygon.points.append(p1)
        polygon.polygon.points.append(p2)
        polygon.polygon.points.append(p3)
        polygon.polygon.points.append(p4)

        self.pub.publish(polygon)


    def inside_polygon(self, x, y):
        """
        Return True if a coordinate (x, y) is inside a polygon defined by
        a list of verticies [(x1, y1), (x2, x2), ... , (xN, yN)].

        Reference: http://www.ariel.com.au/a/python-point-int-poly.html
        """
        points = self.bound_box_points

        n = len(points)
        inside = False
        p1x, p1y = points[0]
        for i in range(1, n + 1):
            p2x, p2y = points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """

        now = detected_clusters_msg.header.stamp
       
        accepted_clusters = []

        evidences = PersonEvidenceArray()
        evidences.header = detected_clusters_msg.header

        for i,cluster in enumerate(detected_clusters_msg.legs):         
           
            in_bounding_box =  self.inside_polygon(cluster.position.x, cluster.position.y)
            
            
            if in_bounding_box:
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = i
                # Text showing person's ID number 
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                marker.type = Marker.TEXT_VIEW_FACING
                marker.text = "{}\n({:.2f},{:.2f})".format(self.fixed_frame,cluster.position.x, cluster.position.y)
                marker.scale.z = 0.1         
                marker.pose.position.x = cluster.position.x
                marker.pose.position.y = cluster.position.y        
                marker.pose.position.z = 0.5 
                marker.lifetime = rospy.Duration(0.1)
                self.marker_pub.publish(marker)

                # publish rviz markers       
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = i+100
                marker.ns = "detected_legs"                       
                marker.type = Marker.CYLINDER
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.01
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = cluster.position.x
                marker.pose.position.y = cluster.position.y        
                marker.pose.position.z = 0.01                        
                marker.lifetime = rospy.Duration(0.1)

                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)
                
                # save cluster
                accepted_clusters.append(cluster)
        if len(accepted_clusters) <= 1:
            return
        #z = np.array([[complex(c.position.x, c.position.y) for c in accepted_clusters]]) # notice the [[ ... ]])
        
        tree = spatial.KDTree(np.array([[c.position.x, c.position.y] for c in accepted_clusters]))

        for j, pts in enumerate(tree.data):
            nearest_point = tree.query(pts, k=2)
            distance = nearest_point[0][1]
            
            # Save to file1.323
            if self.isSaveToFile:   
                self.f.write(str(distance) +'\n')
                self.f.flush()

            new_evidence = PersonEvidence()

            for i, pt in enumerate([pts]):    
                # publish rviz markers 
                marker = Marker()
                marker.header.frame_id = self.fixed_frame
                marker.header.stamp = now
                marker.id = j + i * 10
                marker.ns = "person"                       
                marker.type = Marker.SPHERE
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                conf = self.leg_context.getProbability(distance)
                marker.color.r = conf
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = pt[0]
                marker.pose.position.y = pt[1]   
                marker.pose.position.z = 0.1                        
                marker.lifetime = rospy.Duration(0.1)
                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)
                
                new_evidence.pose.position.x = (pt[0] + tree.data[nearest_point[1][1]][0])/2.0
                new_evidence.pose.position.y = (pt[1] + tree.data[nearest_point[1][1]][1])/2.0
                new_evidence.confidence = conf

                #rospy.logwarn(self.leg_context.getProbability(out[row][column]))

            evidences.evidences.append(new_evidence)
        
        self.evidence_pub.publish(evidences)


if __name__ == '__main__':
    rospy.init_node('leg_distance_node', anonymous=True)
    rospy.logwarn('THE BOUDING BOX PARAMETERS SHOULD BE THE SAME AS THE ONES IN "extract_positive*.launch" file!')
    #ldistance = LegDistance(x_min=-2.76, x_max=2.47, y_min=-2.37, y_max=1.49)# logfile='/home/airlab/Desktop/newFile.txt')
    
    # TODO move the definition of bounding box to a proper class
    p1 = Point32(2.02451276779, -3.36508965492, 0)
    p2 = Point32(2.88112926483, -0.500505328178, 0)
    p3 = Point32(0.494375228882, 0.16442990303, 0)
    p4 = Point32(-0.135583043098, -2.78914022446, 0)

    
    ldistance = LegDistance(p1, p2, p3, p4)

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        ldistance.publish()
        r.sleep() 
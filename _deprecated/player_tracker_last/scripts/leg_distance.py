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
from geometry_msgs.msg import PolygonStamped, Point32

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
    '''Process pairs of leg clusters and weight them by their
    probability of being a human using a generative model trained
    by data..''' 
    
    def __init__(self, x_min, x_max, y_min, y_max, pub_bounding_box=False, logfile=''):

        self.model = joblib.load(rospy.get_param("leg_dist_model"))

        self.tf_listener = tf.TransformListener()

        self.pub_bounding_box = pub_bounding_box
        self.isSaveToFile = logfile != ''
        self.fixed_frame = rospy.get_param('fixed_frame')

        if self.isSaveToFile:
            self.f = open(logfile, "w")
        
        # accepted area
        self.x_min_ = x_min
        self.x_max_ = x_max
        self.y_min_ = y_min
        self.y_max_ = y_max

        # ROS subscribers         
        self.detected_clusters_sub = rospy.Subscriber('detected_leg_clusters', LegArray, self.detected_clusters_callback)      

        # ROS publishers
        self.pub = rospy.Publisher('bounding_box', PolygonStamped, queue_size=1)
        self.person_evidence_pub = rospy.Publisher('person_evidence_array', PersonEvidenceArray, queue_size=1)
        self.marker_pub = rospy.Publisher('detected_legs_in_bounding_box', Marker, queue_size=5)

        #rospy.spin() # So the node doesn't immediately shut down

    
    def publish(self):
        '''Publish poligon for the bouding box'''
            
        polygon = PolygonStamped()
        polygon.header.stamp = rospy.get_rostime()
        polygon.header.frame_id = 'base_link'
        p1 = Point32()
        p1.x = self.x_min_
        p1.y = self.y_min_
        p2 = Point32()
        p2.x = self.x_max_
        p2.y = self.y_max_

        polygon.polygon.points.append(p1)
        p11 = Point32()
        p11.x = p1.x
        p11.y = p1.y + (p2.y - p1.y)
        p12 = Point32()
        p12.x = p1.x + (p2.x - p1.x)
        p12.y = p1.y
        polygon.polygon.points.append(p1)
        polygon.polygon.points.append(p11)
        polygon.polygon.points.append(p2)
        polygon.polygon.points.append(p12)
        polygon.polygon.points.append(p1)

        self.pub.publish(polygon)

    def getPersonProbability(self,distance):
        '''Uses self.model to calculate the probability of the pair being a person'''
        return np.exp(self.model.score_samples(distance))

    def detected_clusters_callback(self, detected_clusters_msg):    
        """
        Callback for every time detect_leg_clusters publishes new sets of detected clusters. 
        It will try to match the newly detected clusters with tracked clusters from previous frames.
        """

        now = detected_clusters_msg.header.stamp
       
        accepted_clusters = []

        for i,cluster in enumerate(detected_clusters_msg.legs):
            
            in_bounding_box = True
            
            if self.pub_bounding_box:
                in_bounding_box = cluster.position.x > self.x_min_ and \
                                  cluster.position.x < self.x_max_ and \
                                  cluster.position.y > self.y_min_ and \
                                  cluster.position.y < self.y_max_
            
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
                marker.text = "t: {}".format(str(now.to_sec()))
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
        
        tree = spatial.KDTree(np.array([[c.position.x, c.position.y] for c in accepted_clusters]))

        pair_set = set([])

        for j, pts in enumerate(tree.data):
            nearest_point = tree.query(pts, k=2)
            distance = nearest_point[0][1]
            prob =  self.getPersonProbability(distance)[0]

            pair = list(copy.deepcopy(nearest_point[1]))
            pair.sort()
            pair.append(prob) 
            pair = tuple(pair)
            pair_set.add(pair)
            
            # Save to file
            if self.isSaveToFile:   
                self.f.write(str(distance) +'\n')
                self.f.flush()

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
                marker.color.r = prob
                marker.color.g = 0
                marker.color.b = 0
                marker.color.a = 1
                marker.pose.position.x = pt[0]
                marker.pose.position.y = pt[1]        
                marker.pose.position.z = 0.1                        
                marker.lifetime = rospy.Duration(0.1)

                # Publish to rviz and /people_tracked topic.
                self.marker_pub.publish(marker)
        

        evid_msg = PersonEvidenceArray()
        evid_msg.header.frame_id = self.fixed_frame
        evid_msg.header.stamp = now

        for item in pair_set:
            msg = PersonEvidence()
            msg.leg1.x = tree.data[item[0]][0]
            msg.leg1.y = tree.data[item[0]][1]
            msg.leg2.x = tree.data[item[1]][0]
            msg.leg2.y = tree.data[item[1]][1]
            msg.probability = item[2]
            evid_msg.evidences.append(msg)

        self.person_evidence_pub.publish(evid_msg)


if __name__ == '__main__':
    rospy.init_node('leg_distance_node', anonymous=True)
    rospy.logwarn('THE BOUDING BOX PARAMETERS SHOULD BE THE SAME AS THE ONES IN "extract_positive*.launch" file!')
    ldistance = LegContextProcessor(x_min=-2.76, x_max=2.47, y_min=-2.37, y_max=1.49)# logfile='/home/airlab/Desktop/newFile.txt')

    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        if ldistance.pub_bounding_box:
            ldistance.publish()
        r.sleep()
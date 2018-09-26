#!/usr/bin/python
import rospy
import numpy as np
import cv2

from nav_msgs.msg import OccupancyGrid

class BRIEFWrapper(object):
    def __init__(self):
        self.star = cv2.xfeatures2d.StarDetector_create() # Initiate FAST detector
        self.brief = cv2.xfeatures2d.BriefDescriptorExtractor_create() # Initiate BRIEF extractor

    def detectAndCompute(self, image, void):
        keypoints = self.star.detect(image, None)  # find the keypoints with STAR
        keypoints, descriptors = self.brief.compute(image, keypoints) # compute the descriptors with BRIEF
        return keypoints, descriptors

class FREAKWrapper(object):
    def __init__(self):
        self.star = cv2.xfeatures2d.StarDetector_create()
        self.freak = cv2.xfeatures2d.FREAK_create()

    def detectAndCompute(self, image, void):
        keypoints = self.star.detect(image, None)
        keypoints, descriptors = self.freak.compute(image, keypoints)
        return keypoints, descriptors

class PerceptionGridProjector(object):
    keypoints = {}
    descriptors = {}

    def __init__(self, detector_name='ORB'):
        if detector_name == 'ORB':
            self.detector = cv2.ORB_create()
            match_distance = cv2.NORM_HAMMING
        elif detector_name == 'BRIEF':
            self.detector = BRIEFWrapper()
            match_distance = cv2.NORM_HAMMING
        elif detector_name == 'SIFT':
            self.detector = cv2.xfeatures2d.SIFT_create()
            match_distance = cv2.NORM_L2
        elif detector_name == 'SURF':
            self.detector = cv2.xfeatures2d.SURF_create()
            match_distance = cv2.NORM_L2
        elif detector_name == 'FREAK':
            self.detector = FREAKWrapper()
            match_distance = cv2.NORM_HAMMING
        else:
            raise Exception('The detector selected does not exist')
        # create BFMatcher object
        self.matcher = cv2.BFMatcher(match_distance, crossCheck=True)


    def compute_descriptors(self, image):
        kp, des = self.detector.detectAndCompute(image, None)
        return kp, des

    def compute_matches(self, des1, des2):
        matches = self.matcher.match(des2, des1) # inverted order beacuse the second is considered to be the ground-truth or traning
        return sorted(matches, key=lambda x: x.distance)

    def compute_transformation(self, matches, keypoints1, keypoints2):
        pts1, pts2 = self.extract_points(matches, keypoints1, keypoints2)
        M = cv2.getAffineTransform(pts1, pts2)
        M[:, :-1] = self.gs(M[:, :-1]) # normalization of rotation matrix
        return M

    def gs(self, X, row_vecs=True, norm = True):
        if not row_vecs:
            X = X.T
        Y = X[0:1,:].copy()
        for i in range(1, X.shape[0]):
            proj = np.diag((X[i,:].dot(Y.T)/np.linalg.norm(Y,axis=1)**2).flat).dot(Y)
            Y = np.vstack((Y, X[i,:] - proj.sum(0)))
        if norm:
            Y = np.diag(1/np.linalg.norm(Y,axis=1)).dot(Y)
        if row_vecs:
            return Y
        else:
            return Y.T

    def project_points(self, M, points):
        rows = len(points)
        pts = np.reshape(np.float32(points), (rows, 2)) # reshape to be a matrix
        return cv2.warpAffine(pts, M)

    def project_point(self, M, point):
        pt = np.float32([point])
        return cv2.warpAffine(pt, M)

    def project_image(self, M, image):
        return cv2.warpAffine(image, M, (4000,4000))

    def extract_points(self, matches, kp1, kp2):
        pts1 = None
        pts2 = None

        for match in matches:
            pts1 = self.append_point_to(kp1[match.trainIdx], pts1)
            pts2 = self.append_point_to(kp2[match.queryIdx], pts2)
        return pts1[:3, :], pts2[:3, :]

    def append_point_to(self, keypoint, matrix):
        pt = list(keypoint.pt)
        if matrix is None:
            return np.float32([pt])
        else:
            return np.append(matrix, np.float32([pt]), axis=0)

    def get_rotation_matrix(self, map_kp, map_desc, perc_kp, perc_desc):
        matches = self.compute_matches(map_desc, perc_desc)
        rot_matrix = self.compute_transformation(matches, map_kp, perc_kp)
        return rot_matrix

def show_image(window_name, img, click=False):
    cv2.namedWindow(window_name,cv2.WINDOW_NORMAL)
    if click:
        cv2.setMouseCallback(window_name, mouse_callback) #Mouse callback
    cv2.imshow(window_name, img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

   # mouse callback function
def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print x, y


if __name__ == "__main__":

    perception_grid_projector = PerceptionGridProjector('ORB')

    # Load an color image in grayscale
    ### CHANGE PATH FOR FILES ###
    # map_img = cv2.imread('/home/luca/projects/people_tracking/src/leg_tracker/images/image.png',0)
    map_img = cv2.imread('/home/luca/projects/people_tracking/src/leg_tracker/images/image.png', 0)

    show_image('mapimage', map_img)

    # map descriptor
    map_kp, map_desc = perception_grid_projector.compute_descriptors(map_img)

    # image descriptor
    grid_img = cv2.imread('/home/luca/projects/people_tracking/src/leg_tracker/images/image_grid.png', 0)

    show_image('image_grid', grid_img)

    # map descriptor
    perc_kp, perc_desc = perception_grid_projector.compute_descriptors(grid_img)

    rotation_mtx = perception_grid_projector.get_rotation_matrix(perc_kp, perc_desc, map_kp, map_desc)

    trans_ = perception_grid_projector.project_image(rotation_mtx, grid_img)
    show_image('trans_', trans_)
    print trans_.shape

    print "image:  {}".format(np.where(grid_img > 0)[0].shape)
    print "map:    {}".format(np.where(map_img > 0)[0].shape)
    print "trans_: {}".format(np.where(trans_ > 0)[0].shape)
    print "rotation: \n{}".format(np.array_str(rotation_mtx, precision=2, suppress_small=True))


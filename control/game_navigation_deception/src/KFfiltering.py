import numpy as np

class EKF:
    """
    TODO state                                np.array([[0.],           # initial x-position
                                              [0.],           # initial y-position
                                              [0.],           # initial orientation
                                              [0.],           # initial x-velocity
                                              [0.],           # initial y-velocity
                                              [0.]])          # initial angular-velocity

    """

    def __init__(self, init_pose):
        self.n = 6                                   # number of states
        self.q = 0.6                                 # process std
        self.r1 = 0.5                                # position measurement std
        self.r2 = np.pi/36                           # orientation measurement std
        self.r3 = 0.3                                # linear velocity std
        self.r4 = 0.2                                # angular velocity std
        self.T = 0.02                                # sampling time
        self.Q = (self.q**2)*np.eye(self.n)                    # noise covariance matrix of the process (6x6 matrix)
        self.R = np.array([[self.r1**2, 0, 0, 0, 0, 0],            # covariance of measurements
                           [0, self.r1**2, 0, 0, 0, 0],
                           [0, 0, self.r2**2, 0, 0, 0],
                           [0, 0, 0, self.r3**2, 0, 0],
                           [0, 0, 0, 0, self.r3**2, 0],
                           [0, 0, 0, 0, 0, self.r4**2]])

        self.H = np.array([[1., 0, 0, 0, 0, 0],      # Observation Matrix and measurements vector
                           [0, 1., 0, 0, 0, 0],
                           [0, 0, 1., 0, 0, 0],
                           [0, 0, 0, 1., 0, 0],
                           [0, 0, 0, 0, 1., 0],
                           [0, 0, 0, 0, 0, 1.]])

        self.B = np.array([[0, 0, 0],                # Control Matrix and Control inputs
                           [0, 0, 0],
                           [0, 0, 0],
                           [1., 0, 0],
                           [0, 1., 0],
                           [0, 0, 1.]])

        self.robot_estimated_pose = init_pose
        self.P = np.eye(self.n)                                    # initial covariance (6x6 matrix)

    # TODO change name of variable to more interpretable ones.
    def predict(self, robot_pose, U_bar):
        """
        Perform a filtering on the measurements returned from AMCL 
        considering model and measurements uncertainty.
        INPUTS:
            @ robot_pose: a 3D-vector representing the robot pose ([x, y, theta])
            @ U_bar: unsmoothed velocity control commands (from tower navigator and kinect_tracker) -- [ bar_u1, bar_u2, bar_u3] 
        OUTPUTS:
            @ robot_estimated_pose: a 3D-vector representing the robot estimated pose ([x_hat, y_hat, theta_hat])
        """

        # Transition Matrix (theta is in rad)
        chi_x = (-np.sin(robot_pose[2]) / (2 * np.sin(np.pi / 3)) + (np.cos(robot_pose[2]) / (2 + 2 * np.cos(np.pi / 3))))
        chi_y = (-np.cos(robot_pose[2]) / (2 * np.sin(np.pi / 3)) + (np.sin(robot_pose[2]) / (2 + 2 * np.cos(np.pi / 3))))
        self.F = np.array([[1., 0., 0., chi_x*self.T, 0., 0.],
                           [0., 1., 0., 0., chi_y*self.T, 0.],
                           [0., 0., 1., 0., 0., self.T],
                           [0., 0., 0., 1., 0., 0.],
                           [0., 0., 0., 0., 1., 0.],
                           [0., 0., 0., 0., 0., 1.]])

        # A priori prediction
        self.robot_estimated_pose = np.dot(self.F, self.robot_estimated_pose) #+ np.dot(self.B, U_bar)
        self.P = np.dot(np.dot(self.F, self.P), self.F.transpose()) + self.Q
        
        return self.robot_estimated_pose

    def update(self,robot_pose,robot_world_vel):
        """
        Kalman update algorithm
        """
        amcl_meas = np.array([[robot_pose[0]],
                            [robot_pose[1]],
                            [robot_pose[2]],
                            [robot_world_vel[0]],
                            [robot_world_vel[1]],
                            [robot_world_vel[2]]])
    
        Z = np.dot(self.H, amcl_meas)            # measurements vector
        # compute Kalman's Gain
        S =  np.linalg.inv(np.dot(np.dot(self.H,self.P), self.H.transpose())+self.R)
        K = np.dot(np.dot(self.P,self.H.transpose()),S)
        
        # compute measurements residuals
        resid = Z - self.robot_estimated_pose

        # update state and covariance estimates
        self.robot_estimated_pose += np.dot(K,resid)
        self.P = np.dot(np.eye(self.n) - np.dot(K,self.H),self.P)
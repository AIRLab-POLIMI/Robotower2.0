#Class exercise
from Matrix import matrix

measurements = [1,2,3]

x = matrix([[0.],[0.]])             #initial state (location and velocity)
P = matrix([[1000.,0.],[0.,1000.]])  #initial uncertainty
u = matrix([[0.],[0.]])             #external motion
F = matrix([[1.,1.],[0.,1.]])       #next state function
H = matrix([[1., 0.]])              #measurement function
R = matrix([[1.]])                  #measurement uncertainty
I = matrix([[1.,0.],[0.,1.]])       #identity matrix

def filter(x,P):
    #Implements the Kalman Filter function for measurement 
    #   update and prediction step
    
    for n in range(len(measurements)):
        global u,H,F,R,I
        # measurement update
        Z = matrix([[measurements[n]]])
        Y =  Z.transpose() - H * x
        S = H * P * H.transpose() + R
        K = P * H.transpose() * S.inverse()
        x = x + (K*Y)
        P = (I - K*H) * P
        
        # prediction
        x = F * x + u
        P = F * P * F.transpose()
        
        print 'x='
        x.show()
        print 'P='
        P.show()

if __name__ == '__main__':
    filter(x,P)
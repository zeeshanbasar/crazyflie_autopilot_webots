from math import sin, cos, sqrt, atan
import numpy as np

class flightController():
    def __init__(self):
        self.m = 0.05
        self.l = 0.04384
        self.Jr = 6e-5
        self.I = np.array([3.5e-5,3.5e-5,6.25e-5])
        self.alpha = np.array([10.689,2.048,9.535,3.843,2.223,2.177,2,2,0.5,2,0.5,2])
        self.a = np.array([(self.I[1] - self.I[2])/self.I[0],
                           self.Jr/self.I[0],
                           (self.I[2] - self.I[0])/self.I[1],
                           self.Jr/self.I[1],
                           (self.I[0] - self.I[1])/self.I[2]])
        self.b = np.array([self.l/self.I[0], self.l/self.I[1], 1/self.I[2]])
        self.g = 9.81
        self.B = 4e-5
        self.D = 2.4e-6
    
    def backstep(self, desiredStates, currStates):
        omega = 0

        z6 = desiredStates[6] - currStates[6]
        z7 = currStates[7] - desiredStates[7] - self.alpha[6]*z6
        U1 = self.m*(z6 + self.g - self.alpha[6]*(z7 + self.alpha[6]*z6) - self.alpha[7]*z7)/(cos(currStates[0])*cos(currStates[2]))

        z8 = desiredStates[8] - currStates[8]
        z9 = currStates[9] - desiredStates[9] - self.alpha[8]*z8
        ux = self.m*(z8 - self.alpha[8]*(z9 + self.alpha[8]*z8) - self.alpha[9]*z9)/U1
        ux = np.clip(ux, -1, 1)

        z10 = desiredStates[10] - currStates[10]
        z11 = currStates[11] - desiredStates[11] - self.alpha[10]*z10
        uy = self.m*(z10 - self.alpha[10]*(z11 + self.alpha[10]*z10) - self.alpha[11]*z11)/U1
        uy = np.clip(uy, -1, 1)

        pitch_des = atan(ux)
        roll_des = atan(-uy)
        
        z0 = roll_des - currStates[0]
        z1 = currStates[1] - desiredStates[1] - self.alpha[0]*z0
        U2 = (z0 - (self.a[0]*currStates[3]*currStates[5]) - (self.a[1]*currStates[3]*omega) - self.alpha[0]*(z1 + self.alpha[0]*z0) - self.alpha[1]*z1)/self.b[0]
        
        z2 = pitch_des - currStates[2]
        z3 = currStates[3] - desiredStates[3] - self.alpha[2]*z2
        U3 = (z2 - (self.a[2]*currStates[1]*currStates[5]) - (self.a[3]*currStates[1]*omega) - self.alpha[2]*(z3 + self.alpha[2]*z2) - self.alpha[3]*z3)/self.b[1]
        
        z4 = desiredStates[4] - currStates[4]
        z5 = currStates[5] - desiredStates[5] - self.alpha[4]*z4
        U4 = -(z4 - (self.a[4]*currStates[1]*currStates[3]) - self.alpha[4]*(z5 + self.alpha[4]*z4) - self.alpha[5]*z5)/self.b[2]
        
        m1 = sqrt((self.D*U1 - 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        m2 = sqrt((self.D*U1 - 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))
        m3 = sqrt((self.D*U1 + 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        m4 = sqrt((self.D*U1 + 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))

        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        return [m1, m2, m3, m4,[ux,uy],[roll_des,pitch_des]] 


from math import sin, cos, sqrt, atan
import numpy as np

class flightController():
    def __init__(self):
        self.m = 0.05
        self.l = 0.04384
        self.Jr = 6e-5
        self.I = np.array([3.5e-5,3.5e-5,6.25e-5])
        # self.alpha = np.array([10.689,2.048,9.535,3.843,2.223,2.177,2,2,2,2,2,2])
      #   self.alpha = np.array([5,2,5,2,2,2,2,2,2,2,2,2])
        self.a = np.array([(self.I[1] - self.I[2])/self.I[0],
                           self.Jr/self.I[0],
                           (self.I[2] - self.I[0])/self.I[1],
                           self.Jr/self.I[1],
                           (self.I[0] - self.I[1])/self.I[2]])
        self.b = np.array([self.l/self.I[0], self.l/self.I[1], 1/self.I[2]])
        self.g = 9.81
        self.B = 4e-5
        self.D = 2.4e-6

        # self.m_prev = np.array([0,0,0,0])
        self.chi1 = 0.0
        self.chi2 = 0.0
        self.chi3 = 0.0
        self.chi4 = 0.0
        self.chi5 = 0.0
        self.chi6 = 0.0
    
    def integralBackstep(self, desiredStates, currStates, dt):
        
        roll = currStates[0]
        pitch = currStates[2]
        yaw = currStates[4]
        rollRate = currStates[1]
        pitchRate = currStates[3]
        yawRate = currStates[5]
        z = currStates[6]
        x = currStates[8]
        y = currStates[10]
        zDot = currStates[7]
        xDot = currStates[9]
        yDot = currStates[11]

        roll_des = desiredStates[0]
        pitch_des = desiredStates[2]
        yaw_des = desiredStates[4]
        rollRate_des = desiredStates[1]
        pitchRate_des = desiredStates[3]
        yawRate_des = desiredStates[5]
        z_des = desiredStates[6]
        x_des = desiredStates[8]
        y_des = desiredStates[10]
        zDot_des = desiredStates[7]
        xDot_des = desiredStates[9]
        yDot_des = desiredStates[11]


        omega = 0

        ## ALTITUDE
        e7 = z_des - z
        c7 = 3.5
        c8 = 1.5
        lambda4 = 1
        self.chi4 += e7*dt
        e8 = c7*e7 + zDot_des + lambda4*self.chi4 - zDot
        U1 = self.m*(self.g + (1 - c7**2 + lambda4)*e7 + (c7 + c8)*e8 - c7*lambda4*self.chi4)/(cos(roll)*cos(pitch))

        e9 = x_des - x
        c9 = 2
        c10 = 0.5
        lambda5 = 1
        self.chi5 += e7*dt
        e10 = c9*e9 + xDot_des + lambda5*self.chi5 - xDot
        ux = self.m*((1 - c9**2 + lambda5)*e9 + (c9 + c10)*e10 - c9*lambda5*self.chi5)/U1
        ux = np.clip(ux, -1, 1)

        e11 = y_des - y
        c11 = 2
        c12 = 0.5
        lambda6 = 1
        self.chi6 += e11*dt
        e12 = c11*e11 + yDot_des + lambda6*self.chi6 - yDot
        uy = -self.m*((1 - c11**2 + lambda6)*e9 + (c11 + c12)*e12 - c11*lambda6*self.chi6)/U1
        uy = np.clip(uy, -1, 1)

        pitch_des = atan(-ux)
        roll_des = atan(uy)

        ## ROLL
        e1 = roll_des - roll
        e2 = rollRate_des - rollRate
        self.chi1 += e1*dt
        c1 = 10.5
        c2 = 2
        lambda1 = 1
        U2 = ((1 - c1**2 + lambda1)*e1 + (c1 + c2)*e2 - c1*lambda1*self.chi1 + 0 + pitchRate*yawRate*self.a[0] - pitchRate*self.a[1]*omega)/self.b[0]

        ## PITCH
        e3 = pitch_des - pitch
        e4 = pitchRate_des - pitchRate
        self.chi2 += e3*dt
        c3 = 10
        c4 = 2
        lambda2 = 1
        U3 = ((1 - c3**2 + lambda2)*e3 + (c3 + c4)*e4 - c4*lambda2*self.chi2 + 0 + rollRate*yawRate*self.a[2] - rollRate*self.a[3]*omega)/self.b[1]

        ## YAW
        e5 = yaw_des - yaw
        e6 = yawRate_des - yawRate
        self.chi3 += e5*dt
        c5 = 2
        c6 = 2
        lambda3 = 1
        U4 = -((1 - c5**2 + lambda3)*e5 + (c5 + c6)*e6 - c5*lambda3*self.chi3)/self.b[2]

        m1 = sqrt((self.D*U1 - 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        m2 = sqrt((self.D*U1 - 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))
        m3 = sqrt((self.D*U1 + 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        m4 = sqrt((self.D*U1 + 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))

        # m1 = sqrt((self.D*U1 + 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        # m2 = sqrt((self.D*U1 - 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))
        # m3 = sqrt((self.D*U1 - 2*self.D*U3 - self.B*U4)/(4*self.B*self.D))
        # m4 = sqrt((self.D*U1 + 2*self.D*U2 + self.B*U4)/(4*self.B*self.D))

      #   m1 = sqrt((self.D*U1 - 2*self.D*U3 + self.B*U4)/(4*self.B*self.D))
      #   m2 = sqrt((self.D*U1 + 2*self.D*U2 - self.B*U4)/(4*self.B*self.D))
      #   m3 = sqrt((self.D*U1 + 2*self.D*U3 + self.B*U4)/(4*self.B*self.D))
      #   m4 = sqrt((self.D*U1 - 2*self.D*U2 - self.B*U4)/(4*self.B*self.D))

        m1 = np.clip(m1, 0, 600)
        m2 = np.clip(m2, 0, 600)
        m3 = np.clip(m3, 0, 600)
        m4 = np.clip(m4, 0, 600)

        # self.m_prev = np.array([m1,m2,m3,m4])

        # return [m1, m2, m3, m4,[ux,uy],[roll_des,pitch_des]] 
        return m1, m2, m3, m4


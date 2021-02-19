#!/usr/bin/python2.7

import math
import numpy as np

class signature_bot:

    def __init__(self):
        self.a = 0.13614
        self.b = 0.05008
        self.th1 = 0
        self.th2 = 0
        self.d3 = 0
        self.x = 0
        self.y = 0
        self.z = 0
        self.th1_dot = 0
        self.th2_dot = 0
        self.d3_dot = 0
        self.x_dot = 0
        self.y_dot = 0
        self.z_dot = 0
        self.m = np.array([0.039657, 0.17058, 0.13263])
        self.com = np.array([[ 0.000015557, 0.0066616, -0.00013792], [ 0.010254, -0.020237, 0.038712 ], [ 0.0037978, -0.072635, 0.0066217 ]])

        I1 = np.array([[ 0.0000033195, 0, 0 ], [ 0, 0.0000059864, 0 ], [ 0, 0, 0.0000037559 ]])
        I2 = np.array([[ 0.000065188, 0, 0 ], [ 0, 0.000090374, 0 ], [ 0, 0, 0.00012972 ]])
        I3 = np.array([[ 0.00015884, 0, 0 ], [ 0, 0.00012513, -0.00002127 ], [ 0, -0.00002127, 0.000043022 ]])

        self.J1 = np.array([[ 0.5*( I1[1,1] + I1[2,2] - I1[0,0] ), I1[0,1], I1[0,2], self.m[0]*self.com[0,0] ],
                           [ I1[1,0], 0.5*( I1[0,0] + I1[2,2] - I1[1,1] ), I1[1,2], self.m[0]*self.com[0,1] ],
                           [ I1[2,0], I1[2,1], 0.5*( I1[0,0] + I1[1,1] - I1[2,2] ), self.m[0]*self.com[0,2] ],
                           [ self.m[0]*self.com[0,0], self.m[0]*self.com[0,1], self.m[0]*self.com[0,2], self.m[0] ]])

        self.J2 = np.array([[ 0.5*( I2[1,1] + I2[2,2] - I2[0,0] ), I2[0,1], I2[0,2], self.m[1]*self.com[1,0] ],
                           [ I2[1,0], 0.5*( I2[0,0] + I2[2,2] - I2[1,1] ), I2[1,2], self.m[1]*self.com[1,1] ],
                           [ I2[2,0], I2[2,1], 0.5*( I2[0,0] + I2[1,1] - I2[2,2] ), self.m[1]*self.com[1,2] ],
                           [ self.m[1]*self.com[1,0], self.m[1]*self.com[1,1], self.m[1]*self.com[1,2], self.m[1] ]])

        self.J3 = np.array([[ 0.5*( I3[1,1] + I3[2,2] - I3[0,0] ), I3[0,1], I3[0,2], self.m[2]*self.com[2,0] ],
                           [ I3[1,0], 0.5*( I3[0,0] + I3[2,2] - I3[1,1] ), I3[1,2], self.m[2]*self.com[2,1] ],
                           [ I3[2,0], I3[2,1], 0.5*( I3[0,0] + I3[1,1] - I3[2,2] ), self.m[2]*self.com[2,2] ],
                           [ self.m[2]*self.com[2,0], self.m[2]*self.com[2,1], self.m[2]*self.com[2,2], self.m[2] ]])

    def get_fk(self):
        self.x = math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a) - self.b*math.sin(self.th1)
        self.y = -1*math.sin(self.th2)*(self.d3 + self.a)
        self.z = -1*self.b*math.cos(self.th1) - math.cos(self.th2)*math.sin(self.th1)*(self.d3 + self.a)

    def get_ik(self):
        r = math.sqrt( pow(self.x, 2) + pow(self.z, 2) )
        l = math.atan2(self.z, self.x)
        self.th1 = math.asin( -1*self.b / r ) - l
        k = self.x*math.cos(self.th1) - self.z*math.sin(self.th1)
        self.th2 = math.atan2(-1*self.y, k)
        self.d3 = k/math.cos(self.th2) - self.a

    def get_Jv(self):
        Jacobian = np.array([ [-1*math.sin(self.th1)*math.cos(self.th2)*(self.d3 + self.a) - self.b*math.cos(self.th1), -1*math.sin(self.th2)*math.cos(self.th1)*(self.d3 + self.a), math.cos(self.th1)*math.cos(self.th2)],
                            [0, -1*math.cos(self.th2)*(self.d3 + self.a), -1*math.sin(self.th2)],
                            [self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a), math.sin(self.th1)*math.sin(self.th2)*(self.d3 + self.a), -1*math.cos(self.th2)*math.sin(self.th1)] ])
        return Jacobian

    def get_invJv(self):
        Jacobian = self.get_Jv()
        return np.linalg.inv(Jacobian)

    def trajectory_plan(self, initial_pos, final_pos, T, N):
        dt = float(T) / float(N-1)
        plan = np.zeros((6,N))

        #x-axis trajectory
        C_0 = initial_pos[0]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[0] - initial_pos[0] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[0] - initial_pos[0] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[0] - initial_pos[0] )

        for i in range(0, N):
            t = dt*i
            plan[0,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[3,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))

        #y-axis trajectory
        C_0 = initial_pos[1]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[1] - initial_pos[1] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[1] - initial_pos[1] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[1] - initial_pos[1] )

        for i in range(0, N):
            t = dt*i
            plan[1,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[4,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))

        #z-axis trajectory
        C_0 = initial_pos[2]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[2] - initial_pos[2] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[2] - initial_pos[2] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[2] - initial_pos[2] )

        for i in range(0, N):
            t = dt*i
            plan[2,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[5,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))

        return plan, dt

    def inv_vel_kin(self):
        lin_vel = np.array([[self.x_dot], [self.y_dot], [self.z_dot]])
        ang_vel = np.matmul(self.get_invJv(),lin_vel)
        self.th1_dot = ang_vel[0]
        self.th2_dot = ang_vel[1]
        self.d3_dot = ang_vel[2]

    def get_M(self):
        M = np.zeros((3, 3))
        M[0,0] = math.sin(self.th1)*(self.J2[2,2]*math.sin(self.th1) + self.J2[3,2]*self.b*math.sin(self.th1)) + math.cos(self.th1)*(self.J2[2,2]*math.cos(self.th1) + self.J2[3,2]*self.b*math.cos(self.th1)) + self.b*math.cos(self.th1)*(self.J2[2,3]*math.cos(self.th1) \
            - self.J2[1,3]*math.cos(self.th2)*math.sin(self.th1) - self.J2[0,3]*math.sin(self.th1)*math.sin(self.th2) + self.J2[3,3]*self.b*math.cos(self.th1)) + self.b*math.sin(self.th1)*(self.J2[2,3]*math.sin(self.th1) + self.J2[1,3]*math.cos(self.th1)*math.cos(self.th2) \
            + self.J2[0,3]*math.cos(self.th1)*math.sin(self.th2) + self.J2[3,3]*self.b*math.sin(self.th1)) + math.cos(self.th1)*math.cos(self.th2)*(self.J2[1,1]*math.cos(self.th1)*math.cos(self.th2) + self.J2[3,1]*self.b*math.sin(self.th1)) \
            + math.cos(self.th2)*math.sin(self.th1)*(self.J2[1,1]*math.cos(self.th2)*math.sin(self.th1) - self.J2[3,1]*self.b*math.cos(self.th1)) + math.cos(self.th1)*math.sin(self.th2)*(self.J2[0,0]*math.cos(self.th1)*math.sin(self.th2) + self.J2[3,0]*self.b*math.sin(self.th1)) \
            + math.sin(self.th1)*math.sin(self.th2)*(self.J2[0,0]*math.sin(self.th1)*math.sin(self.th2) - self.J2[3,0]*self.b*math.cos(self.th1)) + (self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3))*(self.J3[1,3]*math.sin(self.th1) \
            + self.J3[3,3]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) - self.J3[2,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[0,3]*math.cos(self.th1)*math.sin(self.th2)) + (self.b*math.cos(self.th1) \
            + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3))*(self.J3[1,3]*math.cos(self.th1) + self.J3[3,3]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,3]*math.cos(self.th2)*math.sin(self.th1) \
            - self.J3[0,3]*math.sin(self.th1)*math.sin(self.th2)) + math.cos(self.th1)*(self.J3[1,1]*math.cos(self.th1) + self.J3[3,1]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,1]*math.cos(self.th2)*math.sin(self.th1)) \
            + math.sin(self.th1)*(self.J3[1,1]*math.sin(self.th1) + self.J3[3,1]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) - self.J3[2,1]*math.cos(self.th1)*math.cos(self.th2)) - math.cos(self.th1)*math.cos(self.th2)*(self.J3[1,2]*math.sin(self.th1) \
            + self.J3[3,2]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) - self.J3[2,2]*math.cos(self.th1)*math.cos(self.th2)) + math.cos(self.th2)*math.sin(self.th1)*(self.J3[1,2]*math.cos(self.th1) + self.J3[2,3]*(self.b*math.cos(self.th1) \
            + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,2]*math.cos(self.th2)*math.sin(self.th1)) + math.cos(self.th1)*math.sin(self.th2)*(self.J3[3,0]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) \
            + self.J3[0,0]*math.cos(self.th1)*math.sin(self.th2)) - math.sin(self.th1)*math.sin(self.th2)*(self.J3[3,0]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) - self.J3[0,0]*math.sin(self.th1)*math.sin(self.th2)) \
            + self.J1[0,0]*pow(math.cos(self.th1),2) + self.J1[1,1]*pow(math.cos(self.th1),2) + self.J1[0,0]*pow(math.sin(self.th1),2) + self.J1[1,1]*pow(math.sin(self.th1),2)

        M[0,1] = (self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3))*(self.J3[0,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[2,3]*math.cos(self.th1)*math.sin(self.th2) + self.J3[3,3]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
            + (self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3))*(self.J3[0,3]*math.cos(self.th2)*math.sin(self.th1) + self.J3[2,3]*math.sin(self.th1)*math.sin(self.th2) + self.J3[3,3]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
            + math.cos(self.th1)*(self.J3[2,1]*math.cos(self.th1)*math.sin(self.th2) + self.J3[3,1]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.sin(self.th1)*(self.J3[2,1]*math.sin(self.th1)*math.sin(self.th2) + self.J3[3,1]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
            - math.sin(self.th1)*math.sin(self.th2)*(self.J3[0,0]*math.cos(self.th1)*math.cos(self.th2) + self.J3[3,0]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.cos(self.th2)*math.sin(self.th1)*(self.J3[2,2]*math.cos(self.th1)*math.sin(self.th2) \
            + self.J3[3,2]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.cos(self.th1)*math.sin(self.th2)*(self.J3[0,0]*math.cos(self.th2)*math.sin(self.th1) + self.J3[3,0]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
            - math.cos(self.th1)*math.cos(self.th2)*(self.J3[2,2]*math.sin(self.th1)*math.sin(self.th2) + self.J3[3,2]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + self.b*math.cos(self.th1)*(self.J2[0,3]*math.cos(self.th1)*math.cos(self.th2) \
            - self.J2[1,3]*math.cos(self.th1)*math.sin(self.th2)) + self.b*math.sin(self.th1)*(self.J2[0,3]*math.cos(self.th2)*math.sin(self.th1) - self.J2[2,3]*math.sin(self.th1)*math.sin(self.th2))

        M[0,2] = -1*(self.J3[3,1]*pow(math.cos(self.th1),2)*math.cos(self.th2) + self.J3[3,1]*math.cos(self.th2)*pow(math.sin(self.th1),2) + self.J3[3,3]*math.cos(self.th1)*math.cos(self.th2)*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) \
            + self.J3[3,3]*math.cos(self.th2)*math.sin(self.th1)*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)))

        M[1,0] = math.cos(self.th1)*math.sin(self.th2)*(self.J3[1,2]*math.cos(self.th1) + self.J3[3,2]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,2]*math.cos(self.th2)*math.sin(self.th1)) \
            + math.sin(self.th1)*math.sin(self.th2)*(self.J3[1,2]*math.sin(self.th1) + self.J3[3,2]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) - self.J3[2,2]*math.cos(self.th1)*math.cos(self.th2)) \
            + math.cos(self.th1)*math.cos(self.th2)*(self.J3[3,0]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) - self.J3[0,0]*math.sin(self.th1)*math.sin(self.th2)) + math.cos(self.th2)*math.sin(self.th1)*(self.J3[3,0]*(self.b*math.sin(self.th1) \
            - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) + self.J3[0,0]*math.cos(self.th1)*math.sin(self.th2)) + math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)*(self.J3[1,3]*math.cos(self.th1) + self.J3[3,3]*(self.b*math.cos(self.th1) \
            + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,3]*math.cos(self.th2)*math.sin(self.th1) - self.J3[0,3]*math.sin(self.th1)*math.sin(self.th2)) + math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)*(self.J3[1,3]*math.sin(self.th1) \
            + self.J3[3,3]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) - self.J3[2,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[0,3]*math.cos(self.th1)*math.sin(self.th2)) \
            - (math.cos(self.th1)*math.cos(self.th2)*(self.J2[0,0]*math.sin(self.th1)*math.sin(self.th2) - self.J2[3,0]*self.b*math.cos(self.th1)) - math.cos(self.th1)*math.sin(self.th2)*(self.J2[1,1]*math.cos(self.th2)*math.sin(self.th1) - self.J2[3,1]*self.b*math.cos(self.th1)) \
            - math.cos(self.th2)*math.sin(self.th1)*(self.J2[0,0]*math.cos(self.th1)*math.sin(self.th2) + self.J2[3,0]*self.b*math.sin(self.th1)) + math.sin(self.th1)*math.sin(self.th2)*(self.J2[1,1]*math.cos(self.th1)*math.cos(self.th2) + self.J2[3,1]*self.b*math.sin(self.th1)))

        M[1,1] = math.cos(self.th2)*(self.J3[2,2]*math.cos(self.th2) + self.J3[3,2]*math.cos(self.th2)*(self.a + self.d3)) + math.sin(self.th2)*(self.J3[0,0]*math.sin(self.th2) - self.J3[3,0]*math.cos(self.th2)*(self.a + self.d3)) \
            + math.cos(self.th1)*math.cos(self.th2)*(self.J3[0,0]*math.cos(self.th1)*math.cos(self.th2) + self.J3[3,0]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.cos(self.th2)*(self.a + self.d3)*(self.J3[2,3]*math.cos(self.th2) \
            - self.J3[0,3]*math.sin(self.th2) + self.J3[3,3]*math.cos(self.th2)*(self.a + self.d3)) + math.cos(self.th1)*math.sin(self.th2)*(self.J3[2,2]*math.cos(self.th1)*math.sin(self.th2) + self.J3[3,2]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
            + math.cos(self.th2)*math.sin(self.th1)*(self.J3[0,0]*math.cos(self.th2)*math.sin(self.th1) + self.J3[3,0]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.sin(self.th1)*math.sin(self.th2)*(self.J3[2,2]*math.sin(self.th1)*math.sin(self.th2) \
            + self.J3[3,2]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)*(self.J3[0,3]*math.cos(self.th2)*math.sin(self.th1) + self.J3[2,3]*math.sin(self.th1)*math.sin(self.th2) \
            + self.J3[3,3]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)*(self.J3[0,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[2,3]*math.cos(self.th1)*math.sin(self.th2) \
            + self.J3[3,3]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) + self.J2[0,0]*pow(math.cos(self.th1),2)*pow(math.cos(self.th2),2) + self.J2[1,1]*pow(math.cos(self.th1),2)*pow(math.sin(self.th2),2) + self.J2[0,0]*pow(math.cos(self.th2),2)*pow(math.sin(self.th1),2) \
            + self.J2[1,1]*pow(math.cos(self.th2),2) + self.J2[1,1]*pow(math.sin(self.th1),2)*pow(math.sin(self.th2),2) + self.J2[0,0]*pow(math.sin(self.th2),2)

        M[1,2] = -1*(self.J3[3,0]*pow(math.sin(self.th2),2) + self.J3[3,0]*pow(math.cos(self.th1),2)*pow(math.cos(self.th2),2) + self.J3[3,0]*pow(math.cos(self.th2),2)*pow(math.sin(self.th1),2) - self.J3[3,2]*math.cos(self.th2)*math.sin(self.th2) \
            - self.J3[3,3]*math.cos(self.th2)*math.sin(self.th2)*(self.a + self.d3) + self.J3[3,2]*pow(math.cos(self.th1),2)*math.cos(self.th2)*math.sin(self.th2) + self.J3[3,2]*math.cos(self.th2)*pow(math.sin(self.th1),2)*math.sin(self.th2) \
            + self.J3[3,3]*pow(math.cos(self.th1),2)*math.cos(self.th2)*math.sin(self.th2)*(self.a + self.d3) + self.J3[3,3]*math.cos(self.th2)*pow(math.sin(self.th1),2)*math.sin(self.th2)*(self.a + self.d3))

        M[2,0] = -1*(math.cos(self.th1)*math.cos(self.th2)*(self.J3[1,3]*math.cos(self.th1) + self.J3[3,3]*(self.b*math.cos(self.th1) + math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3)) + self.J3[2,3]*math.cos(self.th2)*math.sin(self.th1) \
            - self.J3[0,3]*math.sin(self.th1)*math.sin(self.th2)) + math.cos(self.th2)*math.sin(self.th1)*(self.J3[1,3]*math.sin(self.th1) + self.J3[3,3]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) \
            - self.J2[2,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[0,3]*math.cos(self.th1)*math.sin(self.th2)))

        M[2,1] = -1*(math.cos(self.th1)*math.cos(self.th2)*(self.J3[0,3]*math.cos(self.th1)*math.cos(self.th2) + self.J3[2,3]*math.cos(self.th1)*math.sin(self.th2) + self.J3[3,3]*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3)) \
        - math.sin(self.th2)*(self.J3[2,3]*math.cos(self.th2) - self.J3[0,3]*math.sin(self.th2) + self.J3[3,3]*math.cos(self.th2)*(self.a + self.d3)) + math.cos(self.th2)*math.sin(self.th1)*(self.J3[0,3]*math.cos(self.th2)*math.sin(self.th1) \
        + self.J3[2,3]*math.sin(self.th1)*math.sin(self.th2) + self.J3[3,3]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3)))

        M[2,2] = self.J3[3,3]*pow(math.cos(self.th1),2)*pow(math.cos(self.th2),2) + self.J3[3,3]*pow(math.cos(self.th2),2)*pow(math.sin(self.th1),2) + self.J3[3,3]*pow(math.sin(self.th2),2)

        return M

    def get_G(self):
        G = np.zeros((3,1))
        g = -9.81;

        G[0,0] = g*self.m[2]*(self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3)) + self.b*g*self.m[1]*math.sin(self.th1) - g*self.m[0]*self.com[0,0]*math.cos(self.th1) \
            + g*self.m[0]*self.com[0,1]*math.sin(self.th1) + g*self.m[2]*self.com[1,1]*math.sin(self.th1) + g*self.m[1]*self.com[1,2]*math.sin(self.th1) + g*self.m[1]*self.com[1,1]*math.cos(self.th1)*math.cos(self.th2) \
            - g*self.m[2]*self.com[1,2]*math.cos(self.th1)*math.cos(self.th2) + g*self.m[1]*self.com[0,1]*math.cos(self.th1)*math.sin(self.th2) + g*self.m[2]*self.com[0,1]*math.cos(self.th1)*math.sin(self.th2)

        G[1,0] = g*self.m[2]*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3) + g*self.m[1]*self.com[0,1]*math.cos(self.th2)*math.sin(self.th1) + g*self.m[2]*self.com[0,1]*math.cos(self.th2)*math.sin(self.th1) \
            - g*self.m[1]*self.com[1,1]*math.sin(self.th1)*math.sin(self.th2) + g*self.m[2]*self.com[1,2]*math.sin(self.th1)*math.sin(self.th2)

        G[2,0] = -g*self.m[2]*math.cos(self.th2)*math.sin(self.th1)

        return G

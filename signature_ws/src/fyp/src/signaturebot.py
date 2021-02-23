#!/usr/bin/python2.7

import math
import numpy as np

class signature_bot:

    def __init__(self):
        self.a = 0.13608
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
        self.th1_ddot = 0
        self.th2_ddot = 0
        self.d3_ddot = 0
        self.x_ddot = 0
        self.y_ddot = 0
        self.z_ddot = 0
        self.th1_eff = 0
        self.th2_eff = 0
        self.d3_eff = 0
        self.m = np.array([0.034759, 0.17499, 0.13248])
        self.com = np.array([[ 1.7832e-5, 0.0063839, 0.00015736], [ -0.019758, 0.009982, 0.087957 ], [ -0.0037892, 0.07262, -0.12942 ]])

        I1 = np.array([[ 2.9791e-6, 0, 0 ], [ 0, 5.3233e-6, 0 ], [ 0, 0, 3.414e-6 ]])
        I2 = np.array([[ 9.0705e-5, 0, 0 ], [ 0, 6.5527e-5, 0 ], [ 0, 0, 0.00013037 ]])
        I3 = np.array([[ 0.00015884, 0, 0 ], [ 0, 0.00012513, 2.127e-5 ], [ 0, 2.127e-5, 4.3022e-5 ]])

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
                            [self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a), math.sin(self.th1)*math.sin(self.th2)*(self.d3 + self.a), -1*math.cos(self.th2)*math.sin(self.th1)] ], dtype=np.float32)
        return Jacobian

    def get_invJv(self):
        Jacobian = self.get_Jv()
        return np.linalg.inv(Jacobian)

    def trajectory_plan(self, initial_pos, final_pos, T, N):
        dt = float(T) / float(N-1)
        plan = np.zeros((9,N))

        #x-axis trajectory
        C_0 = initial_pos[0]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[0] - initial_pos[0] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[0] - initial_pos[0] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[0] - initial_pos[0] )

        for i in range(0, N):
            t = dt*i
            plan[0,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[3,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))
            plan[6,i] = 6*C_3*t + 12*C_4*float(pow(t,2)) + 20*C_5*float(pow(t,3))

        #y-axis trajectory
        C_0 = initial_pos[1]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[1] - initial_pos[1] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[1] - initial_pos[1] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[1] - initial_pos[1] )

        for i in range(0, N):
            t = dt*i
            plan[1,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[4,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))
            plan[7,i] = 6*C_3*t + 12*C_4*float(pow(t,2)) + 20*C_5*float(pow(t,3))

        #z-axis trajectory
        C_0 = initial_pos[2]
        C_3 = ( 10.0 / float(pow(T,3)) ) * ( final_pos[2] - initial_pos[2] )
        C_4 = -1.0*( 15.0 / float(pow(T,4)) ) * ( final_pos[2] - initial_pos[2] )
        C_5 = ( 6.0 / float(pow(T,5)) ) * ( final_pos[2] - initial_pos[2] )

        for i in range(0, N):
            t = dt*i
            plan[2,i] = C_0 + C_3*float(pow(t,3)) + C_4*float(pow(t,4)) + C_5*float(pow(t,5))
            plan[5,i] = 3*C_3*float(pow(t,2)) + 4*C_4*float(pow(t,3)) + 5*C_5*float(pow(t,4))
            plan[8,i] = 6*C_3*t + 12*C_4*float(pow(t,2)) + 20*C_5*float(pow(t,3))

        return plan, dt

    def inv_vel_kin(self):
        lin_vel = np.array([[self.x_dot], [self.y_dot], [self.z_dot]], dtype=np.float32).reshape(3,1)
        ang_vel = np.matmul(self.get_invJv(),lin_vel)
        self.th1_dot = ang_vel[0]
        self.th2_dot = ang_vel[1]
        self.d3_dot = ang_vel[2]

    def get_Jv_dot(self):
        Jacobian =  np.array([ [self.b*self.th1_dot*math.sin(self.th1) - self.d3_dot*math.cos(self.th2)*math.sin(self.th1) - self.th1_dot*math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3) \
                 + self.th2_dot*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3), self.th1_dot*math.sin(self.th1)*math.sin(self.th2)*(self.a + self.d3) \
                 - self.th2_dot*math.cos(self.th1)*math.cos(self.th2)*(self.a + self.d3) - self.d3_dot*math.cos(self.th1)*math.sin(self.th2), self.th1_dot*math.cos(self.th2)*math.sin(self.th1) \
                 - self.th2_dot*math.cos(self.th1)*math.sin(self.th2) ],
                 [0, self.th2_dot*math.sin(self.th2)*(self.a + self.d3) - self.d3_dot*math.cos(self.th2), -self.th2_dot*math.cos(self.th2)],
                 [self.b*self.th1_dot*math.cos(self.th1) - self.d3_dot*math.cos(self.th1)*math.cos(self.th2) + self.th1_dot*math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3) \
                 + self.th2_dot*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3), self.d3_dot*math.sin(self.th1)*math.sin(self.th2) + self.th1_dot*math.cos(self.th1)*math.sin(self.th2)*(self.a + self.d3) \
                 + self.th2_dot*math.cos(self.th2)*math.sin(self.th1)*(self.a + self.d3),   self.th2_dot*math.sin(self.th1)*math.sin(self.th2) - self.th1_dot*math.cos(self.th1)*math.cos(self.th2)] ], dtype=np.float32)

        return Jacobian

    def inv_accel_kin(self):
        lin_accel = np.array([[self.x_ddot], [self.y_ddot], [self.z_ddot]], dtype=np.float32).reshape(3,1)
        ang_vel = np.array([[self.th1_dot], [self.th2_dot], [self.d3_dot]], dtype=np.float32).reshape(3,1)
        temp = lin_accel - np.matmul(self.get_Jv_dot(),ang_vel)
        ang_accel = np.matmul(self.get_invJv(),temp)
        self.th1_ddot = ang_accel[0]
        self.th2_ddot = ang_accel[1]
        self.d3_ddot = ang_accel[2]

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

    def get_efforts(self):
        M = self.get_M()
        G = self.get_G()
        ang_accel = np.array([ [self.th1_ddot],[self.th2_ddot],[self.d3_ddot] ], dtype=np.float32).reshape(3,1)
        temp = np.matmul(M,ang_accel) + G

        self.th1_eff = temp[0]
        self.th2_eff = temp[1]
        self.d3_eff = temp[2]

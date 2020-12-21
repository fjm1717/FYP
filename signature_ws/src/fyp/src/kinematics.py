#!/usr/bin/python2.7

import math
import numpy as np

class signature_bot:

    def __init__(self, th1, th2, d3, x, y, z, th1_dot, th2_dot, d3_dot, x_dot, y_dot, z_dot):
        self.a = 0.134
        self.b = 0.05008
        self.th1 = th1
        self.th2 = th2
        self.d3 = d3
        self.x = x
        self.y = y
        self.z = z
        self.th1_dot = th1_dot
        self.th2_dot = th2_dot
        self.d3_dot = d3_dot
        self.x_dot = x_dot
        self.y_dot = y_dot
        self.z_dot = z_dot

    def get_fk(self):
        self.x = math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a) - self.b*math.sin(self.th1)
        self.y = -1*math.sin(self.th2)*(self.d3 + self.a)
        self.z = -1*self.b*math.cos(self.th1) - math.cos(self.th2)*math.sin(self.th1)*(self.d3 + self.a)

    def get_ik(self):
        self.th1 = math.asin( -1*self.b / math.sqrt( pow(self.x, 2) + pow(self.z, 2) ) ) - math.atan2(self.z, self.x)
        k = self.x*math.cos(self.th1) - self.z*math.sin(self.th1)
        self.th2 = math.atan2(-1*self.y, k)
        self.d3 = k/math.cos(self.th2) - self.a

    def get_Jv(self):
        Jacobian = np.array([ [-1*math.sin(self.th1)*math.cos(self.th2)*(self.d3 + self.a) - self.b*math.cos(self.th1), -1*math.sin(self.th2)*math.cos(self.th1)*(self.d3 + self.a) - self.b*math.sin(self.th1), math.cos(self.th1)*math.cos(self.th2)], [0, -1*math.cos(self.th2)*(self.d3 + self.a), -1*math.sin(self.th2)], [self.b*math.sin(self.th1) - math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a), math.sin(self.th1)*math.sin(self.th2)*(self.d3 + self.a), -1*math.cos(self.th2)*math.sin(self.th1)] ])
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

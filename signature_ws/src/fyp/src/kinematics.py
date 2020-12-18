#!/usr/bin/python2.7

import math

class signature_bot:

    def __init__(self, a, b, th1, th2, d3, x, y, z):
        self.a = a
        self.b = b
        self.th1 = th1
        self.th2 = th2
        self.d3 = d3
        self.x = x
        self.y = y
        self.z = z

    def get_fk(self):
        self.x = math.cos(self.th1)*math.cos(self.th2)*(self.d3 + self.a) - self.b*math.sin(self.th1)
        self.y = -1*math.sin(self.th2)*(self.d3 + self.a)
        self.z = -1*self.b*math.cos(self.th1) - math.cos(self.th2)*math.sin(self.th1)*(self.d3 + self.a)

    def get_ik(self):
        self.th1 = math.asin( -1*self.b / math.sqrt( pow(self.x, 2) + pow(self.z, 2) ) ) - math.atan2(self.z, self.x)
        k = self.x*math.cos(self.th1) - self.z*math.sin(self.th1)
        self.th2 = math.atan2(-1*self.y, k)
        self.d3 = k/math.cos(self.th2) - self.a

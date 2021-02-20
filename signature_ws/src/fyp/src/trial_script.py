#!/usr/bin/python2.7

import rospy
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

robot = signaturebot.signature_bot()
print(robot.J1)
print(robot.J2)
print(robot.J3)

print(robot.com)
print(robot.m)

robot.th1 = 0;
robot.th2 = 0;
robot.d3 = 0.02;

M = robot.get_M()
print(M)
G = robot.get_G()
print(G)

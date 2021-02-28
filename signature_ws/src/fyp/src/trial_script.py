#!/usr/bin/python2.7

import rospy
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

rospy.init_node('trial')

robot = signaturebot.signature_bot()

pose = np.zeros((3,1))
centre = np.array([[0.175],[0],[-0.05]])
radius = 0.015

robot.th1 = 0.5
robot.th2 = 0
robot.d3 = 0
robot.get_fk()

pose[0] = robot.x
pose[1] = robot.y
pose[2] = robot.z

dx = ( centre - pose ) * ( 1 - ( radius / np.linalg.norm(centre-pose) ) )
print(dx)

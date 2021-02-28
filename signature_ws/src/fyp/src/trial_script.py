#!/usr/bin/python2.7

import rospy
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

rospy.init_node('trial')

robot = signaturebot.signature_bot()

robot.th1 = 0.5
robot.th2 = 0
robot.d3 = 0
robot.get_fk()

G = robot.get_G()
print('Gravity: ')
print(G)

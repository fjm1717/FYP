#!/usr/bin/python2.7

import rospy
import time
import sys
import math
import signaturebot
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

error = 0.0

def error_reader(msg):
    global error

    try:
        error = msg.data
    except:
        print('Nope')

rospy.init_node('trial')

pub = rospy.Publisher('signaturebot/position/command', JointState, queue_size=10)
sub = rospy.Subscriber('signaturebot/position/error', Float64, error_reader)

robot = signaturebot.signature_bot()
rate = rospy.Rate(5)

command = JointState()

th1 = -20*(math.pi/180)
th2 = 0*(math.pi/180)
d3 = 0.02

time.sleep(2)

command.position = [th1, th2, d3]
pub.publish(command)

while not rospy.is_shutdown():
    print('Error: ' + str(error))

    rate.sleep()

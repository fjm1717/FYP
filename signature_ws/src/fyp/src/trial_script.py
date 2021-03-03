#!/usr/bin/python2.7

import rospy
import time
import sys
import math
import signaturebot
import numpy as np
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

error = 0.0

def error_reader(msg):
    global error

    try:
        error = msg.data
    except:
        print('Nope')

rospy.init_node('trial')

pub = rospy.Publisher('signaturebot/force_position/command', Twist, queue_size=10)
sub = rospy.Subscriber('signaturebot/force_position/error', Float64, error_reader)

robot = signaturebot.signature_bot()
rate = rospy.Rate(50)

i = 0.0
dt = 0.02

command = Twist()
command.linear.y = 0.0
command.linear.z = -0.05008

while not rospy.is_shutdown():

    command.linear.x = 0.15 + 0.01*math.sin(i/50)
    pub.publish(command)

    i = i + dt

    print('Error: ' + str(error))
    rate.sleep()

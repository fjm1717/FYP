#!/usr/bin/python2.7

import rospy
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

rospy.init_node('effort_trajectory')

pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

rate = rospy.Rate(10)

i = 0
while not rospy.is_shutdown():
    pub1.publish(i)
    pub2.publish(i)
    pub3.publish(i)
    i = i + 0.01;

    rate.sleep()

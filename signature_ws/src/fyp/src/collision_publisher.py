#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from gazebo_msgs.msg import ContactState
import std_msgs.msg

contact = np.zeros(3)

def contact_reader(msg):
    global contact
    contact[0] = msg.states.depths[0]
    contact[1] = msg.states.depths[1]
    contact[2] = msg.states.depths[2]

#subscribe to collision_detection to monitor recieve ContactState upon base collision
joint_sub = rospy.Subscriber('/collision_detection',ContactState,contact_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)
rospy.init_node('workspace_sweep')

rate = rospy.Rate(10)

#250,000 joint space locations
th1_range = np.linspace(-1.5708, 1.5708, 50, endpoint=True)
th2_range = np.linspace(-1.5708, 1.5708, 50, endpoint=True)
d3_range = np.linspace(0, 0.096, 100, endpoint=True)

ws = np.zeros(50,50,100)

for i in th1_range:
    for j in th2_range:
        for k in d3_range:

            pitch_pub.publish(i)
            yaw_pub.publish(j)
            ext_pub.publish(k)

            rate.sleep()

            ws[i,j,k] =


while not rospy.is_shutdown():
    rate.sleep()

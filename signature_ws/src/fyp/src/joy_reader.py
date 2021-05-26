#!/usr/bin/python2.7

import rospy
import math
import sys
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

xbox = np.zeros(6)
state = np.zeros(6)

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #z-axis
    xbox[1] = float(msg.axes[1]) #x-axis
    xbox[2] = float(msg.axes[0]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

rospy.init_node('joy_reader')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = signaturebot.signature_bot()
robot.get_fk()

rate = rospy.Rate(50)

print('------------Joy Demonstration-----------')

while not rospy.is_shutdown():
    #loop until node shutdown

    if xbox[3] == 1:
        #set home position if A button press
        robot.th1 = 0.0
        robot.th2 = 0.0
        robot.d3 = 0.0

    #set axis positions using controller input
    robot.th1 = robot.th1 - xbox[0] * 0.01
    robot.th2 = robot.th2 + xbox[2] * 0.01
    robot.d3 = robot.d3 + xbox[1] * 0.001
    robot.get_fk()

    #publish joint variables to arm_controller/position/joint/command
    pitch_pub.publish(robot.th1)
    yaw_pub.publish(robot.th2)
    ext_pub.publish(robot.d3)

    print('Current State: ' + 'th1: ' + str(round(robot.th1,2)) + ' th2: ' + str(round(robot.th2,2)) + ' d3: ' + str(round(robot.d3,3)))
    sys.stdout.write("\033[F")
    sys.stdout.write("\033[K")

    rate.sleep()

#!/usr/bin/python2.7

import rospy
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import sys
import time

xbox = np.zeros(6)
radius = 0.02

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #z-axis
    xbox[1] = float(msg.axes[1]) #x-axis
    xbox[2] = float(msg.axes[0]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

rospy.init_node('spherical_constraint')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = signaturebot.signature_bot()
robot.get_fk()


time.sleep(3)

print('-------------Spherical Constraint----------------')
print('-------------------------------------------------')
print('Moving to Centre..')

robot.x = 0.16
robot.y = 0
robot.z = -0.05008
robot.get_ik()

pitch_pub.publish(robot.th1)
yaw_pub.publish(robot.th2)
ext_pub.publish(robot.d3)

rate = rospy.Rate(50)

print('---------------Current Position------------------')
print('-------------------------------------------------')

while not rospy.is_shutdown():
    #loop until node shutdown

    if xbox[3] == 1:
        #set home position if A button press
        robot.x = 0.16
        robot.y = 0.0
        robot.z = -0.05008

    #set axis positions using controller input
    robot.z = robot.z + xbox[0] * 0.001
    robot.x = robot.x + xbox[1] * 0.001
    robot.y = robot.y + xbox[2] * 0.001

    #spherical constraints
    if pow(robot.x - 0.16,2) >= ( pow(radius,2) - pow(robot.y,2) - pow(robot.z + 0.05008,2) ):
        robot.x = robot.x - xbox[1] * 0.001
    if pow(robot.y,2) >= ( pow(radius,2) - pow(robot.x - 0.16,2) - pow(robot.z + 0.05008,2) ):
        robot.y = robot.y - xbox[2] * 0.001
    if pow(robot.z + 0.05008,2) >= ( pow(radius,2) - pow(robot.x - 0.16,2) - pow(robot.y,2) ):
        robot.z = robot.z - xbox[0] * 0.001

    #if ( pow(robot.x - 0.16,2) + pow(robot.y,2) + pow(robot.z + 0.05008,2) >= pow(radius,2) ):
    #    robot.x = 0.16
    #    robot.y = 0
    #    robot.z = -0.05008

    robot.get_ik()

    #publish joint variables to arm_controller/position/joint/command
    pitch_pub.publish(robot.th1)
    yaw_pub.publish(robot.th2)
    ext_pub.publish(robot.d3)

    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    sys.stdout.write("\033[F")

    rate.sleep()

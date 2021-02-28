#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header, Float64

state = np.zeros(13)
xbox = np.zeros(6)

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        time = msg.header.stamp
        state[0] = round(float(msg.position[0]),6)
        state[1] = round(float(msg.position[1]),6)
        state[2] = round(float(msg.position[2]),6)
        state[3] = round(float(msg.velocity[0]),6)
        state[4] = round(float(msg.velocity[1]),6)
        state[5] = round(float(msg.velocity[2]),6)
        state[9] = round(float(msg.effort[0]),6)
        state[10] = round(float(msg.effort[1]),6)
        state[11] = round(float(msg.effort[2]),6)
        state[12] = time.to_sec()
    except:
        pass

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #th1
    xbox[1] = float(msg.axes[1]) #d3
    xbox[2] = float(msg.axes[0]) #th2
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

rospy.init_node('position_command')
pub1 = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_reader)

rate = rospy.Rate(50)
time.sleep(2)

robot = signaturebot.signature_bot()

robot.th1 = 0.0
robot.th2 = 0.0
robot.d3 = 0.0

pub1.publish(robot.th1)
pub2.publish(robot.th2)
pub3.publish(robot.d3)

while not rospy.is_shutdown():

    if xbox[3] == 1:
        #set home position if A button press
        robot.th1 = 0.0
        robot.th2 = 0.0
        robot.d3 = 0.0
        robot.get_fk()

    robot.th1 = robot.th1 + xbox[0]*0.005
    robot.th2 = robot.th2 + xbox[2]*0.005
    robot.d3 =  robot.d3 + xbox[1]*0.001
    robot.get_fk()

    G_target = robot.get_G()

    pub1.publish(robot.th1)
    pub2.publish(robot.th2)
    pub3.publish(robot.d3)

    print('--------------------------------------')
    print('Target EE: ' + str(robot.x) + ' ' + str(robot.y) + ' ' + str(robot.z))
    print('Target Joint Positions: ' + str(robot.th1) + ' ' + str(robot.th2) + ' ' + str(robot.d3))

    print('--------------------------------------')
    print('Joint Positions: ' + str(state[2]) + ' ' + str(state[1]) + ' ' + str(state[0]))
    print('Joint Velocities: ' + str(state[5]) + ' ' + str(state[4]) + ' ' + str(state[3]))
    print('Gazebo Efforts: ' + str(state[11]) + ' ' + str(state[10]) + ' ' + str(state[9]))

    print('--------------------------------------')
    print('Target Gravity: ' + str(G_target[0]) + ' ' + str(G_target[1]) + ' ' + str(G_target[2]))
    #print('Actual Gravity: ' + str(G[0]) + ' ' + str(G[1]) + ' ' + str(G[2]))
    print('')

    rate.sleep()

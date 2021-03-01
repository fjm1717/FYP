#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64

state = np.zeros(13)
exe_rate = 100

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        #joints published alphabetically (ext, pitch, yaw)
        time = msg.header.stamp
        state[0] = round(float(msg.position[1]),6)
        state[1] = round(float(msg.position[2]),6)
        state[2] = round(float(msg.position[0]),6)
        state[3] = round(float(msg.velocity[1]),6)
        state[4] = round(float(msg.velocity[2]),6)
        state[5] = round(float(msg.velocity[0]),6)
        state[6] = round(float(msg.effort[1]),6)
        state[7] = round(float(msg.effort[2]),6)
        state[8] = round(float(msg.effort[0]),6)
        state[9] = time.to_sec()
    except:
        pass

rospy.init_node('effort_position')
rate = rospy.Rate(exe_rate)

#float publishers to arm_controller/effort/joint/command to set joint efforts
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('--------Effort Position Control--------')
rate = rospy.Rate(exe_rate)

#move pitch joint towards home position before starting viscoelastic force control
while not rospy.is_shutdown():
    print('---------------------------------------')
    robot.d3 = state[2]
    robot.get_fk()

    print('')
    print('Current Position: ')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('pitch: ' + str(robot.th1) + ' yaw: ' + str(robot.th2) + ' ext: ' + str(robot.d3))

    robot.th1 = input('Set th1 position: ')*(math.pi/180)
    robot.th2 = input('Set th2 position: ')*(math.pi/180)
    pitch_pub.publish(robot.th1)
    yaw_pub.publish(robot.th2)

    G = robot.get_G()
    print('Gravity Efforts: ' + str(G[0]) + ' ' + str(G[1]) + ' ' + str(G[2]))
    eff = input('Set d3 effort: ')
    print('')

    ext_pub.publish(eff)

    rate.sleep()

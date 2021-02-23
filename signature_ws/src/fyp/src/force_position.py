#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64
from matplotlib import pyplot as plt

state = np.zeros(13)
exe_rate = 50

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        time = msg.header.stamp
        state[0] = float(msg.position[0])
        state[1] = float(msg.position[1])
        state[2] = float(msg.position[2])
        state[3] = float(msg.velocity[0])
        state[4] = float(msg.velocity[1])
        state[5] = float(msg.velocity[2])
        state[9] = float(msg.effort[0])
        state[10] = float(msg.effort[1])
        state[11] = float(msg.effort[2])
        state[12] = time.to_sec()

    except:
        pass

rospy.init_node('force_position')
rate = rospy.Rate(exe_rate)

pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('--------Force Position Control--------')

robot.x = 0.16
robot.y = 0.02
robot.z = -0.04
robot.get_ik()
target_th1 = robot.th1
target_th2 = robot.th2
target_d3 = robot.d3

print('--------------------------------------')
print('Moving to position..')

#IK to move manipulator to starting pose
target = np.array([[robot.x],[robot.y],[robot.z]], dtype=float)
error_signal = np.array([[1],[1],[1]], dtype=float)
last_error_signal = error_signal
k = 1.0
kp = 60.0
kd = 1.2;

#viscoelastic force control loop
while(1):
    robot.th1 = state[0]
    robot.th2 = state[1]
    robot.d3 = state[2]
    robot.get_fk()

    print('--------------------------------------')
    current_pose = np.array([[robot.x],[robot.y],[robot.z]], dtype=float).reshape(3,1)
    print('Current Pose: ' + str(robot.x) + ' ' + str(robot.y) + ' ' + str(robot.z))
    error_signal = ( k * ( target - current_pose ) ).reshape(3,1)
    print('Target: ' + str(target[0]) + ' ' + str(target[1]) + ' ' + str(target[2]))
    diff_error_signal = ( exe_rate * ( error_signal - last_error_signal ) ).reshape(3,1)
    print('Error Signal: ')
    print(error_signal)
    force = ( (kp*error_signal) + (kd*diff_error_signal) ).reshape(3,1)
    print('Force: ')
    print(force)
    efforts = np.matmul(np.transpose(robot.get_Jv()),force) + robot.get_G()
    print('Efforts: ')
    print(efforts)

    pub1.publish(efforts[0])
    pub2.publish(efforts[1])
    pub3.publish(efforts[2])

    last_error_signal = error_signal

    rate.sleep()

while not rospy.is_shutdown():
    pass

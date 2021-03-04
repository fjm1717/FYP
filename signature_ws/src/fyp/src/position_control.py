#!/usr/bin/python2.7

import rospy
import time
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist

state = np.zeros(3)
target = np.zeros((3,1))
exe_rate = 100

controller = False

robot = signaturebot.signature_bot()

def command_reader(msg):
    global target

    rospy.loginfo("Received a /command message!")
    rospy.loginfo("Joint Positions: [%f, %f, %f]"%(msg.position[0], msg.position[1], msg.position[2]))

    try:
        target[0] = float(msg.position[0])
        target[1] = float(msg.position[1])
        target[2] = float(msg.position[2])
    except:
        print('Target Error')

def joint_reader(msg):
    global state

    rospy.loginfo("Received a /state message!")
    rospy.loginfo("Joint Positions: [%f, %f, %f]"%(msg.position[1], msg.position[2], msg.position[0]))

    try:
        #joints published alphabetically (ext, pitch, yaw)
        state[0] = round(float(msg.position[1]),6)
        state[1] = round(float(msg.position[2]),6)
        state[2] = round(float(msg.position[0]),6)
    except:
        print('Joint State Error')

rospy.init_node('position_control')
rate = rospy.Rate(exe_rate)

#publishers to arm_controller/effort/joint/command to set joint efforts
pitch = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=10)
yaw = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=10)
ext = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=10)

#publish rms error values during PID loop
rms_error = rospy.Publisher('signaturebot/position/error', Float64, queue_size=10)

#subscribe to /position/command to recieve target position
command_sub = rospy.Subscriber('signaturebot/position/command', JointState, command_reader)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

kp = np.diag([1.5,1.5,16.0])
kd = np.diag([0.02,0.02,1.2])
ki = np.diag([0.2,0.2,10.0])

pose = np.zeros((3,1))
diff_error = np.zeros((3,1))
last_error = np.zeros((3,1))
error = np.zeros((3,1))
int_error = np.zeros((3,1))
efforts = np.zeros((3,1))

dt = 1.0 / exe_rate

print('----------Force Position----------')

while not rospy.is_shutdown():

    #controller turned off if target = 0,0,0
    if (np.count_nonzero(target) != 0):
        controller = True

        robot.th1 = state[0]
        robot.th2 = state[1]
        robot.d3 = state[2]

        pose[0] = robot.th1
        pose[1] = robot.th2
        pose[2] = robot.d3

        G = robot.get_G()

        error = target - pose
        diff_error = ( error - last_error ) / dt
        int_error = int_error + error*dt

        rms = math.sqrt(np.mean(np.square(error)))
        rms_error.publish(rms)

        last_error = error

        efforts = np.matmul(kp,error) + np.matmul(kd,diff_error) + np.matmul(ki,int_error)

        #publish efforts to gazebo
        pitch.publish(efforts[0])
        yaw.publish(efforts[1])
        ext.publish(efforts[2] + G[2])

        rate.sleep()

    else:
        if (controller == True):
            pitch.publish(0.0)
            yaw.publish(0.0)
            ext.publish(0.0)

            controller = False

        rate.sleep()

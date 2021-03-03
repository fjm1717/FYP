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
import message_filters

state = np.zeros(3)
target = np.zeros((3,1))
exe_rate = 50

controller = False

def command_reader(msg):
    global target

    rospy.loginfo("Received a /command message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    try:
        target[0] = float(msg.linear.x)
        target[1] = float(msg.linear.y)
        target[2] = float(msg.linear.z)
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

rospy.init_node('force_position')
rate = rospy.Rate(exe_rate)

#publishers to arm_controller/effort/joint/command to set joint efforts
pitch = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=10)
yaw = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=10)
ext = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=10)

#publish rms error values during PID loop
rms_error = rospy.Publisher('signaturebot/force_position/error', Float64, queue_size=10)

#subscribe to /force_position/command to recieve target position
command_sub = rospy.Subscriber('signaturebot/force_position/command', Twist, command_reader)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

kp = np.diag([10.5,14.5,17.5])
kd = np.diag([1.0,1.0,1.2])
ki = np.diag([2.2,2.2,2.2])

pose = np.zeros((3,1))
diff_error = np.zeros((3,1))
last_error = np.zeros((3,1))
error = np.zeros((3,1))
int_error = np.zeros((3,1))
force = np.zeros((3,1))
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
        robot.get_fk()

        pose[0] = robot.x
        pose[1] = robot.y
        pose[2] = robot.z

        error = target - pose
        diff_error = ( error - last_error ) / dt
        int_error = int_error + error*dt

        rms = math.sqrt(np.mean(np.square(error)))
        rms_error.publish(rms)

        last_error = error

        force = np.matmul(kp,error) + np.matmul(kd,diff_error) + np.matmul(ki,int_error)
        G = robot.get_G()
        efforts = np.matmul(np.transpose(robot.get_Jv()),force)

        efforts = efforts + G

        #publish efforts to gazebo
        pitch.publish(efforts[0])
        yaw.publish(efforts[1])
        ext.publish(efforts[2])

        rate.sleep()

    else:
        if (controller == True):
            pitch.publish(0.0)
            yaw.publish(0.0)
            ext.publish(0.0)

            controller = False

        rate.sleep()

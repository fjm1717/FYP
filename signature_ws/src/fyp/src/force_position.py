#!/usr/bin/python2.7

import rospy
import time
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist, WrenchStamped, Wrench, Vector3

state = np.zeros(3)
target = np.zeros((3,1))
exe_rate = 50

controller = False

def command_reader(msg):
    global target

    rospy.loginfo("Received a /command message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))

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

#publish force vector at end effector to visualise in rviz
wrench_pub = rospy.Publisher('signaturebot/wrench', WrenchStamped, queue_size=10)

robot = signaturebot.signature_bot()

kp = np.diag([12.0,15.0,18.65])
kd = np.diag([1.0,1.0,2.0])
ki = np.diag([0.5,0.5,1.8])

pose = np.zeros((3,1))
diff_error = np.zeros((3,1))
last_error = np.zeros((3,1))
error = np.zeros((3,1))
int_error = np.zeros((3,1))
force = np.zeros((3,1))
efforts = np.zeros((3,1))

dt = 1.0 / exe_rate

#set up wrench msg
wrench_msg = WrenchStamped()
wrench_msg.header.frame_id = "extension_link"
wrench_msg.wrench.torque.x = 0.0
wrench_msg.wrench.torque.y = 0.0
wrench_msg.wrench.torque.z = 0.0

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

        #publish wrench msg
        wrench_msg.header.stamp = rospy.get_rostime()
        #transformed from world to end effector ref frame
        ee_force = np.matmul(robot.get_invR(),force)
        wrench_msg.wrench.force.x = ee_force[0]
        wrench_msg.wrench.force.y = ee_force[1]
        wrench_msg.wrench.force.z = ee_force[2]
        wrench_pub.publish(wrench_msg)

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

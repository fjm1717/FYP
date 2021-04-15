#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header, Float64

path = Path()
show = 0

def odom_path(msg):
    global path
    global show
    if (show != 0):
        try:
            path.header = msg.header
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            path.poses.append(pose)
            path_pub.publish(path)
        except:
            print('Path Republish Error')

exe_rate = 200

rospy.init_node('position_trajectory')

#float publishers to arm_controller/effort/joint/command to set joint efforts
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=10)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=10)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=10)

#publisher and subscriber for recieving robot pose and sending to rviz to visualise path
odom_sub = rospy.Subscriber('/odom', Odometry, odom_path)
path_pub = rospy.Publisher('/path', Path, queue_size=10)

robot = signaturebot.signature_bot()

time.sleep(3)
print('-------------Path Visualisation----------------')

initial_pos = np.array([0.16, -0.15, -0.08], dtype="float")
final_pos = np.array([0.16, 0.15, -0.02], dtype="float")

rate = rospy.Rate(exe_rate)

T = 10
N = 500

plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)

#IK to move manipulator to starting pose
robot.x = initial_pos[0]
robot.y = initial_pos[1]
robot.z = initial_pos[2]
robot.get_ik()

pitch_pub.publish(robot.th1)
yaw_pub.publish(robot.th2)
ext_pub.publish(robot.d3)

print('-----------------------------------------------')
print('Moving to start position..')

time.sleep(3)

joint_pos = np.zeros((3,N))
planned_time = np.linspace(0,T,N)

print('Planning Trajectory..')

for i in range(0,N):
    #find joint states from cartesian positions along trajectory
    robot.x = plan[0,i]
    robot.y = plan[1,i]
    robot.z = plan[2,i]
    robot.get_ik()

    #store joint space trajectory data
    joint_pos[0,i] = robot.th1
    joint_pos[1,i] = robot.th2
    joint_pos[2,i] = robot.d3

    rate.sleep()

print('Executing Trajectory..')

#begin path visualisation in rviz
show = 1

for i in range(0,N):
    pitch_pub.publish(joint_pos[0,i])
    yaw_pub.publish(joint_pos[1,i])
    ext_pub.publish(joint_pos[2,i])

    time.sleep(dt)

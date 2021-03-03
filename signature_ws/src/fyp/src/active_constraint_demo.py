#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist

state = np.zeros(10)
xbox = np.zeros(6)
rms_error = 1.0
exe_rate = 50

#spherical constraints
centre = np.array([[0.175],[0.0],[-0.05]])
radius = 0.02

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #z-axis
    xbox[1] = float(msg.axes[1]) #x-axis
    xbox[2] = float(msg.axes[0]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

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

def error_reader(msg):
    global rms_error

    try:
        rms_error = msg.data
    except:
        pass

rospy.init_node('active_constraint_demo')
rate = rospy.Rate(exe_rate)

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy', Joy, xbox_reader)

#float publishers to arm_controller/effort/joint/command to set joint efforts
eff_pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
eff_pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
eff_pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

position_pub = rospy.Publisher('signaturebot/force_position/command', Twist, queue_size=10)
error_sub = rospy.Subscriber('signaturebot/force_position/error', Float64, error_reader)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('--------Active Constraint Demo--------')

time.sleep(2)

print('--------------------------------------')
print('Moving EE into Boundary Centre..')

command = Twist()
command.linear.x = centre[0]
command.linear.y = centre[1]
command.linear.z = centre[2]

position_pub.publish(command)
while(rms_error > 1e-2):
    pass

command.linear.x = 0.0
command.linear.y = 0.0
command.linear.z = 0.0
#turn off position controller
position_pub.publish(command)

#user force gains
kf = np.diag([0.3,0.3,0.4])
#elastic force gains
ke = np.diag([10.0,10.0,15.0])

pose = np.zeros((3,1))
force = np.zeros((3,1))
efforts = np.zeros((3,1))

while not rospy.is_shutdown():

    #store current location of robot in joint and task space
    robot.th1 = state[0]
    robot.th2 = state[1]
    robot.d3 = state[2]
    robot.get_fk()

    pose[0] = robot.x
    pose[1] = robot.y
    pose[2] = robot.z

    print('--------------------------------------')
    print('EE Position: ' + str(robot.x) + ' ' + str(robot.y) + ' ' + str(robot.z))
    #print('Joint Positions: ' + str(robot.th1) + ' ' + str(robot.th2) + ' ' + str(robot.d3))

    #end-effector force from user input
    user_input = np.array([[xbox[1]],[xbox[2]],[xbox[0]]])
    force = np.matmul(kf,user_input)
    G = robot.get_G()
    #print('User Force: ' + str(force[0]) + ' ' + str(force[1]) + ' ' + str(force[2]))
    #print('Gravity: ' + str(G[0]) + ' ' + str(G[1]) + ' ' + str(G[2]))

    #active constraint
    elastic_efforts = np.zeros((3,1))
    elastic_force = np.zeros((3,1))
    if ( ( pow(robot.x - centre[0],2) + pow(robot.y - centre[1],2) + pow(robot.z - centre[2],2) ) >= pow(radius,2) ):
        #outside boundary
        dx = ( centre - pose ) * ( 1 - ( radius / np.linalg.norm(centre-pose) ) )
        elastic_force = np.matmul(ke,dx)
        elastic_efforts = np.matmul(np.transpose(robot.get_Jv()),elastic_force)

    dist = np.linalg.norm(centre-pose)
    print('Distance from Centre: ' + str(dist) + ' Radius: ' + str(radius))
    print('Elastic Force: ' + str(elastic_force[0]) + ' ' + str(elastic_force[1]) + ' ' + str(elastic_force[2]))

    if (xbox[3]==1):
        efforts = elastic_efforts
    else:
        efforts = elastic_efforts + np.matmul(np.transpose(robot.get_Jv()),force) + G

    #print('Total Efforts: ' + str(efforts[0]) + ' ' + str(efforts[1]) + ' ' + str(efforts[2]))
    #print('Gazebo Efforts: ' + str(state[9]) + ' ' + str(state[10]) + ' ' + str(state[11]))
    print(' ')

    #publish efforts to gazebo
    eff_pub1.publish(efforts[0])
    eff_pub2.publish(efforts[1])
    eff_pub3.publish(efforts[2])

    rate.sleep()

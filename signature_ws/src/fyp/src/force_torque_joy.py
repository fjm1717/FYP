#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64

xbox = np.zeros(6)
state = np.zeros(6)

def joint_reader(msg):
    global state
    state[0] = float(msg.position[0])
    state[1] = float(msg.position[1])
    state[2] = float(msg.position[2])
    state[3] = float(msg.velocity[0])
    state[4] = float(msg.velocity[1])
    state[5] = float(msg.velocity[2])

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #z-axis
    xbox[1] = -1*float(msg.axes[0]) #x-axis
    xbox[2] = float(msg.axes[1]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

rospy.init_node('force_torque')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_reader)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states',JointState,joint_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = kinematics.signature_bot(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

print('-----------------------------')
print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
print('-----------------------------')
print('th1: ' + str(robot.th1) + ' th2: ' + str(robot.th2) + ' d3: ' + str(robot.d3))
print('-----------------------------')

rate = rospy.Rate(50)

cart_F = np.array([[0.0], [0.0], [0.0]])
joint_M = np.array([[0.0], [0.0], [0.0]])

while not rospy.is_shutdown():
    #loop until node shutdown

    if xbox[3] == 1:
        #set force to zero if A button press
        cart_F[0] = 0.0
        cart_F[1] = 0.0
        cart_F[2] = 0.0

    #adjust force vector components using controller input
    cart_F[2] = cart_F[2] + xbox[0] * 0.1 #z-component
    cart_F[0] = cart_F[0] + xbox[1] * 0.1 #x-component
    cart_F[1] = cart_F[1] + xbox[2] * 0.1 #y-component
    print(cart_F)

    #retrieve current robot pose
    robot.th1 = state[0]
    robot.th2 = state[1]
    robot.d3 = state[2]
    robot.get_fk()

    #calculate required joint torques/forces fron jacobian transpose
    joint_M = np.matmul(np.transpose(robot.get_Jv()),cart_F)

    #publish joint variables to arm_controller/position/joint/command
    pitch_pub.publish(joint_M[0])
    yaw_pub.publish(joint_M[1])
    ext_pub.publish(joint_M[2])

    print('----------------Cartesian Force------------------')
    print('x: ' + str(cart_F[0]) + ' y: ' + str(cart_F[1]) + ' z: ' + str(cart_F[2]))
    print('--------------Joint Torques/Forces---------------')
    print('th1: ' + str(joint_M[0]) + ' th2: ' + str(joint_M[1]) + ' d3: ' + str(joint_M[2]))
    print('-------------------------------------------------')

    rate.sleep()

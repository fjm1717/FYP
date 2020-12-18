#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64

z_shift = 0.0
x_shift = 0.0
y_shift = 0.0
zero_state = 0
set_state1 = 0
set_state2 = 0

state = np.zeros(6)

def xbox_input(msg):
    global z_shift
    global x_shift
    global y_shift
    global zero_state
    global set_state1
    global set_state2
    z_shift = float(msg.axes[4])
    x_shift = -1*float(msg.axes[1])
    y_shift = -1*float(msg.axes[0])
    zero_state = int(msg.buttons[0])
    set_state1 = int(msg.buttons[1])
    set_state2 = int(msg.buttons[2])

def joint_reader(msg):
    global state
    state[0] = float(msg.position[0])
    state[1] = float(msg.position[1])
    state[2] = float(msg.position[2])
    state[3] = float(msg.velocity[0])
    state[4] = float(msg.velocity[1])
    state[5] = float(msg.velocity[2])

rospy.init_node('position_trajectory')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_input)
joint_sub = rospy.Subscriber('signaturebot/joint_states',JointState,joint_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/velocity/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/velocity/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/velocity/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008, 0, 0 ,0 ,0 ,0 ,0)

initial_pos = np.array([0.0, 0.0, 0.0], dtype="float")
final_pos = np.array([0.0, 0.0, 0.0], dtype="float")

pitch_pub.publish(0.0)
yaw_pub.publish(0.0)
ext_pub.publish(0.0)

while not rospy.is_shutdown():
    #loop until node shutdown

    rate = rospy.Rate(50)

    T = input('Input Time to Complete Trajectory (s)..')
    N = input('Input Number of Points along Trajectory..')

    print('Set Inital Position (Press B)')

    while set_state1 == 0:
    #loop until position set

        #set axis velocities using controller input
        robot.z_dot = z_shift * 0.001
        robot.x_dot = x_shift * 0.001
        robot.y_dot = y_shift * 0.001
        print('Cart. Speeds: ' + str(robot.x_dot) + ' ' + str(robot.y_dot) + ' ' + str(robot.z_dot))

        #get current joint positions from /joint_states
        robot.th1 = state[0]
        robot.th2 = state[1]
        robot.d3 = state[2]
        robot.get_fk()

        joint_dot = np.matmul(robot.get_invJv(),np.array([[robot.x_dot],[robot.y_dot],[robot.z_dot]]))
        robot.th1_dot = joint_dot[0]
        robot.th2_dot = joint_dot[1]
        robot.d3_dot = joint_dot[2]

        #publish joint variables to arm_controller/velocity/joint/command
        pitch_pub.publish(robot.th1_dot)
        yaw_pub.publish(robot.th2_dot)
        ext_pub.publish(robot.d3_dot)
        print('Joint Speeds: ' + str(robot.th1_dot) + ' ' + str(robot.th2_dot) + ' ' + str(robot.d3_dot))

        rate.sleep()

    print('------------Initial Position Set------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('--------------------------------------------')

    initial_pos[0] = robot.x
    initial_pos[1] = robot.y
    initial_pos[2] = robot.z

    print('Set Final Position (Press X)')

    while set_state2 == 0:
    #loop until position set

        #set axis velocities using controller input
        robot.z_dot = z_shift * 0.001
        robot.x_dot = x_shift * 0.001
        robot.y_dot = y_shift * 0.001

        #get current joint positions from /joint_states
        robot.th1 = state[0]
        robot.th2 = state[1]
        robot.d3 = state[2]
        robot.get_fk()

        joint_dot = np.matmul(robot.get_invJv(),np.array([[robot.x_dot],[robot.y_dot],[robot.z_dot]]))
        robot.th1_dot = joint_dot[0]
        robot.th2_dot = joint_dot[0]
        robot.d3_dot = joint_dot[0]

        #publish joint variables to arm_controller/velocity/joint/command
        pitch_pub.publish(robot.th1_dot)
        yaw_pub.publish(robot.th2_dot)
        ext_pub.publish(robot.d3_dot)

        rate.sleep()

    print('-------------Final Position Set-------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('--------------------------------------------')

    final_pos[0] = robot.x
    final_pos[1] = robot.y
    final_pos[2] = robot.z

    #plan sraight line trajectory

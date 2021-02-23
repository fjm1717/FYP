#!/usr/bin/python2.7

import rospy
import math
import signaturebot
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
    xbox[1] = float(msg.axes[1]) #x-axis
    xbox[2] = float(msg.axes[0]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

def print_end_effector(state):
    print('------------------------Joint State--------------------------')
    print('th1: ' + str(state[0]) + ' th2: ' + str(state[1]) + ' d3: ' + str(state[2]))
    print('th1_dot: ' + str(state[3]) + ' th2_dot: ' + str(state[4]) + ' d3_dot: ' + str(state[5]))
    print('-------------------------------------------------------------')


rospy.init_node('position_trajectory')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_reader)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states',JointState,joint_reader)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing positions/speeds in joint & physical space to use in kinematics
robot = signaturebot.signature_bot()
robot.get_fk()

initial_pos = np.array([0.0, 0.0, 0.0], dtype="float")
final_pos = np.array([0.0, 0.0, 0.0], dtype="float")

while not rospy.is_shutdown():
    #loop until node shutdown

    rate = rospy.Rate(50)

    T = input('Input Time to Complete Trajectory (s)..')
    N = input('Input Number of Points along Trajectory..')

    print('Set Inital Position (Press B)')

    while xbox[4] == 0:
    #loop until position set

        if xbox[3] == 1:
            #set home position if A button press
            robot.th1 = 0.0
            robot.th2 = 0.0
            robot.d3 = 0.0
            robot.get_fk()

        #set axis positions using controller input
        robot.z = robot.z + xbox[0] * 0.001
        robot.x = robot.x - xbox[1] * 0.001
        robot.y = robot.y + xbox[2] * 0.001

        robot.get_ik()

        #publish joint variables to arm_controller/position/joint/command
        pitch_pub.publish(robot.th1)
        yaw_pub.publish(robot.th2)
        ext_pub.publish(robot.d3)

        rate.sleep()

    print('------------------Initial Position Set-----------------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('-------------------------------------------------------------')

    initial_pos[0] = robot.x
    initial_pos[1] = robot.y
    initial_pos[2] = robot.z

    print('Set Final Position (Press X)')

    while xbox[5] == 0:
    #loop until position set

        if xbox[3] == 1:
            #set home position if A button press
            robot.th1 = 0.0
            robot.th2 = 0.0
            robot.d3 = 0.0
            robot.get_fk()

        #set axis positions using controller input
        robot.z = robot.z + xbox[0] * 0.001
        robot.x = robot.x - xbox[1] * 0.001
        robot.y = robot.y + xbox[2] * 0.001

        robot.get_ik()

        #joint limits!
        if robot.th1 > 1.57:
            robot.th1 = 1.57
        elif robot.th1 < -1.57:
            robot.th1 = -1.57
        elif robot.th2 > 1.57:
            robot.th2 = 1.57
        elif robot.th2 < -1.57:
            robot.th2 = -1.57
        elif robot.d3 > 0.06:
            robot.d3 = 0.06
        elif robot.d3 < 0:
            robot.d3 = 0

        robot.get_fk()

        #publish joint variables to arm_controller/position/joint/command
        pitch_pub.publish(robot.th1)
        yaw_pub.publish(robot.th2)
        ext_pub.publish(robot.d3)

        rate.sleep()

    print('--------------------Final Position Set-----------------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('-------------------------------------------------------------')

    final_pos[0] = robot.x
    final_pos[1] = robot.y
    final_pos[2] = robot.z

    #plan sraight line trajectory
    plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)
    rate = rospy.Rate(1.0 / dt)
    print('--------------Trajectory Planning Complete-------------------')
    print(' ')

    for i in range(0,N):
        robot.x = plan[0,i]
        robot.y = plan[1,i]
        robot.z = plan[2,i]
        robot.get_ik()

        robot.x_dot = plan[3,i]
        robot.y_dot = plan[4,i]
        robot.z_dot = plan[5,i]
        robot.inv_vel_kin()

        pitch_pub.publish(robot.th1)
        yaw_pub.publish(robot.th2)
        ext_pub.publish(robot.d3)

        print('-----------------------Kinematic Data------------------------')
        print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
        print('x_dot: ' + str(robot.x_dot) + ' y_dot: ' + str(robot.y_dot) + ' z_dot: ' + str(robot.z_dot))
        print('-------------------------------------------------------------')
        print('th1: ' + str(robot.th1) + ' th2: ' + str(robot.th2) + ' d3: ' + str(robot.d3))
        print('th1_dot: ' + str(robot.th1_dot) + ' th2_dot: ' + str(robot.th2_dot) + ' d3_dot: ' + str(robot.d3_dot))
        print('-------------------------------------------------------------')
        print(' ')

        #print_end_effector(state)

        rate.sleep()

#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

z_shift = 0.0
x_shift = 0.0
y_shift = 0.0
zero_state = 0
set_state1 = 0
set_state2 = 0

def callback(msg):
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

rospy.init_node('position_trajectory')

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,callback)

#float publishers to arm_controller/position/joint/command to set joint positions
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008, 0, 0 ,0 ,0 ,0 ,0)

initial_pos = np.array([0.0, 0.0, 0.0], dtype="float")
final_pos = np.array([0.0, 0.0, 0.0], dtype="float")

while not rospy.is_shutdown():
    #loop until node shutdown

    rate = rospy.Rate(50)

    T = input('Input Time to Complete Trajectory (s)..')
    N = input('Input Number of Points along Trajectory..')

    print('Set Inital Position (Press B)')

    while set_state1 == 0:
    #loop until position set

        if zero_state == 1:
            #set home position if A button press
            robot.th1 = 0.0
            robot.th2 = 0.0
            robot.d3 = 0.0
            robot.get_fk()

        #set axis positions using controller input
        robot.z = robot.z + z_shift * 0.001
        robot.x = robot.x - x_shift * 0.001
        robot.y = robot.y + y_shift * 0.001

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

    print('------------Initial Position Set------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('--------------------------------------------')

    initial_pos[0] = robot.x
    initial_pos[1] = robot.y
    initial_pos[2] = robot.z

    print('Set Final Position (Press X)')

    while set_state2 == 0:
    #loop until position set

        if zero_state == 1:
            #set home position if A button press
            robot.th1 = 0.0
            robot.th2 = 0.0
            robot.d3 = 0.0
            robot.get_fk()

        #set axis positions using controller input
        robot.z = robot.z + z_shift * 0.001
        robot.x = robot.x - x_shift * 0.001
        robot.y = robot.y + y_shift * 0.001

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

    print('-------------Final Position Set-------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('--------------------------------------------')

    final_pos[0] = robot.x
    final_pos[1] = robot.y
    final_pos[2] = robot.z

    #plan sraight line trajectory
    plan, dt = robot.position_trajectory_plan(initial_pos, final_pos, T, N)
    rate = rospy.Rate(1.0 / dt)
    print('--------Trajectory Planning Complete--------')

    for i in range(0,N):
        robot.x = plan[0,i]
        robot.y = plan[1,i]
        robot.z = plan[2,i]
        robot.get_ik()

        pitch_pub.publish(robot.th1)
        yaw_pub.publish(robot.th2)
        ext_pub.publish(robot.d3)

        rate.sleep()

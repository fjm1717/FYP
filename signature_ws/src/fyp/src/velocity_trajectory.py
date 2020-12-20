#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
import std_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

rospy.init_node('velocity_trajectory')

#setup means of commanding effort interface to set position, velocity etc.
command = JointTrajectory()
target = JointTrajectoryPoint()
command.joint_names = ['joint_p_1', 'joint_y_1', 'joint_s_1']

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy',Joy,xbox_input)
joint_sub = rospy.Subscriber('signaturebot/joint_states',JointState,joint_reader)

#float publishers to joint trajectory controller to set joint positions, velocties etc.
pub = rospy.Publisher('signaturebot/trajectory/command', JointTrajectory, queue_size=1)

#setup signaturebot class containing geometry plus positions/speeds in joint & physical space
robot = kinematics.signature_bot(0, 0, 0, 0, 0, 0, 0, 0 ,0 ,0 ,0 ,0)
robot.get_fk()

initial_pos = np.array([0.0, 0.0, 0.0], dtype="float")
final_pos = np.array([0.0, 0.0, 0.0], dtype="float")

#setup joint trajectory command and move to home position
target.positions = [0.0, 0.0, 0.0]
target.velocities = [0.8, 0.8, 0.4]
target.time_from_start = rospy.Duration(1)
command.points.append(target)

#allow time for gazebo to boot
rate = rospy.Rate(0.5)
rate.sleep()

command.header.stamp = rospy.get_rostime()
pub.publish(command)

while not rospy.is_shutdown():
    #loop until node shutdown

    rate = rospy.Rate(50)
    target.time_from_start = rospy.Duration(0.001)

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

        #publish joint variables to trajectory/command
        command.points[0].positions = [robot.th1, robot.th2, robot.d3]
        command.header.stamp = rospy.get_rostime()

        pub.publish(command)

        #wait till position reached before publishing next command
        while (abs(state[0] - robot.th1) > 1e-3  or abs(state[1] - robot.th2) > 1e-3 or abs(state[2] - robot.d3) > 1e-3):
            print('Traversing..')

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

        #publish joint variables to trajectory/command
        command.points[0].positions = [robot.th1, robot.th2, robot.d3]
        command.header.stamp = rospy.get_rostime()

        pub.publish(command)

        rate.sleep()

    print('-------------Final Position Set-------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('--------------------------------------------')

    final_pos[0] = robot.x
    final_pos[1] = robot.y
    final_pos[2] = robot.z

    #plan sraight line trajectory
    plan, dt = robot.position_trajectory_plan(initial_pos, final_pos, T, N)
    target.time_from_start = rospy.Duration(dt)
    print('--------Trajectory Planning Complete--------')

    for i in range(0,N):
        robot.x = plan[0,i]
        robot.y = plan[1,i]
        robot.z = plan[2,i]
        robot.get_ik()

        target.positions[0] = [robot.th1, robot.th2, robot.d3]
        command.header.stamp = rospy.get_rostime()

        pub.publish(command)

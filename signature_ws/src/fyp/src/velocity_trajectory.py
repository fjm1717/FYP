#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

state = np.zeros(4)

def joint_reader(msg):
    global state
    try:
        state[0] = float(msg.effort[0])
        state[1] = float(msg.effort[1])
        state[2] = float(msg.effort[2])
        state[3] = float(msg.header.stamp)
    except:
        pass

rospy.init_node('Trial')
pub = rospy.Publisher('signaturebot/trajectory/command', JointTrajectory, queue_size=1)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states',JointState,joint_reader)

robot = kinematics.signature_bot(0,0,0,0,0,0,0,0,0,0,0,0)

command = JointTrajectory()
target = JointTrajectoryPoint()
command.joint_names = ['pitch', 'yaw', 'extension']

initial_pos = np.array([0.157, -0.055, -0.02146], dtype="float")
final_pos = np.array([0.157, 0.0948, -0.0597], dtype="float")

rate = rospy.Rate(20)

T = input('Input Time to Complete Trajectory (s)..')
N = input('Input Number of Points along Trajectory..')

plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)
target.time_from_start = rospy.Duration(dt)

target.positions = [0.0, 0.0, 0.0]
target.velocities = [0.0, 0.0, 0.0]

command.points.append(target)

for i in range(0,N):
    #find joint states from cartesian positions along trajectory
    robot.x = plan[0,i]
    robot.y = plan[1,i]
    robot.z = plan[2,i]
    robot.get_ik()

    #find joint velocities from cartesian velocities along trajectory
    robot.x_dot = plan[3,i]
    robot.y_dot = plan[4,i]
    robot.z_dot = plan[5,i]
    robot.inv_vel_kin()

    command.points[0].positions = [robot.th1, robot.th2, robot.d3]
    command.points[0].velocities = [robot.th1_dot, robot.th2_dot, robot.d3_dot]

    command.header.stamp = rospy.get_rostime()
    pub.publish(command)

    print('-----------------------------')
    print('effort 1: ' + str(state[0]) + ' effort 2: ' + str(state[1]) + ' effort 3: ' + str(state[2]) + ' time: ' + str(state[3]))

    rate.sleep()

print('-----------------------------')
rate.sleep()

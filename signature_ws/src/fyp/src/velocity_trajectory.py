#!/usr/bin/python2.7

import rospy
import time
import csv
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

state = np.zeros(10)
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/output.csv'

def joint_reader(msg):
    global state
    time = rospy.Duration()
    try:
        time = msg.header.stamp
        state[0] = float(msg.position[0])
        state[1] = float(msg.position[1])
        state[2] = float(msg.position[2])
        state[3] = float(msg.velocity[0])
        state[4] = float(msg.velocity[1])
        state[5] = float(msg.velocity[2])
        state[6] = float(msg.effort[0])
        state[7] = float(msg.effort[1])
        state[8] = float(msg.effort[2])
        state[9] = time.to_sec()
    except:
        pass

rospy.init_node('trajectory_data')
pub = rospy.Publisher('signaturebot/trajectory/command', JointTrajectory, queue_size=1)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = kinematics.signature_bot(0,0,0,0,0,0,0,0,0,0,0,0)

command = JointTrajectory()
target = JointTrajectoryPoint()
command.joint_names = ['pitch', 'yaw', 'extension']

initial_pos = np.array([0.1414, -0.02073, -0.1356], dtype="float")
final_pos = np.array([0.1539, 0.03576, 0.02833], dtype="float")

rate = rospy.Rate(20)

T = input('Input Time to Complete Trajectory (s)..')
N = input('Input Number of Points along Trajectory..')

plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)
target.time_from_start = rospy.Duration(dt)

#IK to move manipulator to starting pose
robot.x = initial_pos[0]
robot.y = initial_pos[1]
robot.z = initial_pos[2]
robot.get_ik()

target.positions = [robot.th1, robot.th2, robot.d3]
target.velocities = [0.0, 0.0, 0.0]

command.points.append(target)

command.header.stamp = rospy.get_rostime()
pub.publish(command)

print(' ')
print('Moving to start position..')

time.sleep(1)

joint_pos = np.zeros((3,N))
joint_vel = np.zeros((3,N))
measured_joint_eff = np.zeros((3,N))
measured_joint_pos = np.zeros((3,N))
measured_joint_vel = np.zeros((3,N))
time = np.zeros((1,N))

print('Planning Trajectory..')

#plan trajectory
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

    #store joint space trajectory data
    joint_pos[0,i] = robot.th1
    joint_pos[1,i] = robot.th2
    joint_pos[2,i] = robot.d3

    joint_vel[0,i] = robot.th1_dot
    joint_vel[1,i] = robot.th2_dot
    joint_vel[2,i] = robot.d3_dot

    rate.sleep()

print('Executing..')

#execute trajectory using JointTrajectoryController
start = rospy.get_time()
print(' ')

for i in range(0,N):
    command.points[0].positions = [joint_pos[0,i], joint_pos[1,i], joint_pos[2,i]]
    command.points[0].velocities = [joint_vel[0,i], joint_vel[1,i], joint_vel[2,i]]

    command.header.stamp = rospy.get_rostime()
    pub.publish(command)

    #store joint state
    measured_joint_pos[0,i] = state[0]
    measured_joint_pos[1,i] = state[1]
    measured_joint_pos[2,i] = state[2]
    measured_joint_vel[0,i] = state[3]
    measured_joint_vel[1,i] = state[4]
    measured_joint_vel[2,i] = state[5]

    #store effort data of dynamic simulation
    measured_joint_eff[0,i] = state[6]
    measured_joint_eff[1,i] = state[7]
    measured_joint_eff[2,i] = state[8]

    #store simulation time
    time[0,i] = state[9]

    print('-----------------------------')
    print('effort 1: ' + str(measured_joint_eff[0,i]) + ' effort 2: ' + str(measured_joint_eff[1,i]) + ' effort 3: ' + str(measured_joint_eff[2,i]) + ' time: ' + str(time[0,i]))

    rate.sleep()

print('-----------------------------')

time_elapsed = time[0,N-1] - start
print('Total Time: ' + str(time_elapsed))

print(' ')
print('Exporting data to csv..')

with open(output_path, mode='w') as csv_file:
    data = ['x','y','z','x_dot','y_dot','z_dot','th1','th2','d3','th1_dot','th2_dot','d3_dot','measured_th1','measured_th2','measured_d3','measured_th1_dot','measured_th2_dot','measured_d3_dot','eff_1','eff_2','eff_3','time']
    writer = csv.DictWriter(csv_file, fieldnames=data)

    writer.writeheader()
    for i in range(0,N):
        writer.writerow({'x': str(plan[0,i]), 'y': str(plan[1,i]), 'z': str(plan[2,i]), 'x_dot': str(plan[3,i]), 'y_dot': str(plan[4,i]), 'z_dot': str(plan[5,i]),
        'th1': str(joint_pos[0,i]), 'th2': str(joint_pos[1,i]), 'd3': str(joint_pos[2,i]),
        'th1_dot': str(joint_vel[0,i]), 'th2_dot': str(joint_vel[1,i]), 'd3_dot': str(joint_vel[2,i]),
        'measured_th1': str(measured_joint_pos[0,i]),'measured_th2': str(measured_joint_pos[1,i]),'measured_d3': str(measured_joint_pos[2,i]),
        'measured_th1_dot': str(measured_joint_vel[0,i]),'measured_th2_dot': str(measured_joint_eff[1,i]),'measured_d3_dot': str(measured_joint_vel[2,i]),
        'eff_1': str(measured_joint_eff[0,i]), 'eff_2': str(measured_joint_eff[1,i]), 'eff_3': str(measured_joint_eff[2,i]), 'time': str(time[0,i])})

        rate.sleep()

print('Export Complete.')

while not rospy.is_shutdown():
    pass

#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt

state = np.zeros(10)
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/output.csv'
exe_rate = 200

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
#publish to joint trajectroy coontroller to set joint position and velocities at waypoints.
pub = rospy.Publisher('signaturebot/trajectory/command', JointTrajectory, queue_size=1)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

command = JointTrajectory()
target = JointTrajectoryPoint()
command.joint_names = ['pitch', 'yaw', 'extension']

print('--------------------------------------')

sample_rate = input('Sample Rate: ')

plane = input('Plane? (0: XY / 1: YZ): ')
if plane == 0:
    #XY
    initial_pos = np.array([0.16, -0.1, -0.05008], dtype="float")
    final_pos = np.array([0.14, 0.1, -0.05008], dtype="float")
else:
    #YZ
    initial_pos = np.array([0.16, 0.05, -0.08], dtype="float")
    final_pos = np.array([0.16, -0.15, 0.02], dtype="float")

print('--------------------------------------')

rate = rospy.Rate(exe_rate)

T = input('Input Time to Complete Trajectory (s): ')
N = input('Input Number of Points along Trajectory: ')

plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)

#IK to move manipulator to starting pose
robot.x = initial_pos[0]
robot.y = initial_pos[1]
robot.z = initial_pos[2]
robot.get_ik()

target.time_from_start = rospy.Duration(0.5)
target.positions = [robot.th1, robot.th2, robot.d3]
target.velocities = [0.0, 0.0, 0.0]

command.points.append(target)

command.header.stamp = rospy.get_rostime()
pub.publish(command)

print('--------------------------------------')
print('Moving to start position..')

time.sleep(1)

joint_pos = np.zeros((3,N))
joint_vel = np.zeros((3,N))
joint_accel = np.zeros((3,N))

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

    #find joint accelerations from cartesian velocities along trajectory
    robot.x_ddot = plan[6,i]
    robot.y_ddot = plan[7,i]
    robot.z_ddot = plan[8,i]
    robot.inv_accel_kin()

    joint_accel[0,i] = robot.th1_ddot
    joint_accel[1,i] = robot.th2_ddot
    joint_accel[2,i] = robot.d3_ddot

    rate.sleep()

print('Uploading Trajectory..')

#execute trajectory using JointTrajectoryController, starting once waypoints uploaded
start = rospy.get_rostime() + rospy.Duration(1 + math.ceil(N/exe_rate))

#setup joint trajectory message
trajectory = JointTrajectory()
target = JointTrajectoryPoint()
trajectory.joint_names = ['pitch', 'yaw', 'extension']

#time to begin trajectory
trajectory.header.stamp = start

#define waypoints
for i in range(0,N):
    #time to execute waypoint relative to start
    target_c = copy.deepcopy(target)
    target_c.time_from_start = rospy.Duration(i*dt)
    target_c.positions = [joint_pos[0,i], joint_pos[1,i], joint_pos[2,i]]
    target_c.velocities = [joint_vel[0,i], joint_vel[1,i], joint_vel[2,i]]
    target_c.accelerations = [joint_accel[0,i], joint_accel[1,i], joint_accel[2,i]]

    trajectory.points.append(target_c)

    rate.sleep()

#send trajectory to controller
pub.publish(trajectory)

print('Executing Trajectory..')
print('--------------------------------------')

data_points = T*sample_rate + 2
i = 0

#measured data arrays
measured_joint_eff = np.zeros((3,data_points))
measured_joint_pos = np.zeros((3,data_points))
measured_joint_vel = np.zeros((3,data_points))
time = np.zeros((1,data_points+1))

rate = rospy.Rate(sample_rate)

#wait until time for trajectory to begin
while rospy.get_time() < start.to_sec():
    pass

#take data during execution
while i < data_points:
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
    time[0,i] = state[9] - start.to_sec()

    i+=1
    rate.sleep()

rate = rospy.Rate(exe_rate)

print('Total Time: ' + str(time[0,data_points-1]))

print(' ')
print('Exporting data to csv..')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    data = ['x','y','z','x_dot','y_dot','z_dot','th1','th2','d3','th1_dot','th2_dot','d3_dot','measured_th1','measured_th2','measured_d3','measured_th1_dot','measured_th2_dot','measured_d3_dot','eff_1','eff_2','eff_3','time']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    if data_points > N:
        for i in range(0,data_points):
            if i < N:
                writer.writerow({'x': str(plan[0,i]), 'y': str(plan[1,i]), 'z': str(plan[2,i]), 'x_dot': str(plan[3,i]), 'y_dot': str(plan[4,i]), 'z_dot': str(plan[5,i]),
                'th1': str(joint_pos[0,i]), 'th2': str(joint_pos[1,i]), 'd3': str(joint_pos[2,i]),
                'th1_dot': str(joint_vel[0,i]), 'th2_dot': str(joint_vel[1,i]), 'd3_dot': str(joint_vel[2,i]), 'measured_th1': str(measured_joint_pos[0,i]),'measured_th2': str(measured_joint_pos[1,i]),'measured_d3': str(measured_joint_pos[2,i]),
                'measured_th1_dot': str(measured_joint_vel[0,i]),'measured_th2_dot': str(measured_joint_eff[1,i]),'measured_d3_dot': str(measured_joint_vel[2,i]),
                'eff_1': str(measured_joint_eff[0,i]), 'eff_2': str(measured_joint_eff[1,i]), 'eff_3': str(measured_joint_eff[2,i]), 'time': str(time[0,i])})
            else:
                writer.writerow({'measured_th1': str(measured_joint_pos[0,i]),'measured_th2': str(measured_joint_pos[1,i]),'measured_d3': str(measured_joint_pos[2,i]),
                'measured_th1_dot': str(measured_joint_vel[0,i]),'measured_th2_dot': str(measured_joint_eff[1,i]),'measured_d3_dot': str(measured_joint_vel[2,i]),
                'eff_1': str(measured_joint_eff[0,i]), 'eff_2': str(measured_joint_eff[1,i]), 'eff_3': str(measured_joint_eff[2,i]), 'time': str(time[0,i])})

            rate.sleep()
    else:
        for i in range(0,N):
            if i < data_points:
                writer.writerow({'x': str(plan[0,i]), 'y': str(plan[1,i]), 'z': str(plan[2,i]), 'x_dot': str(plan[3,i]), 'y_dot': str(plan[4,i]), 'z_dot': str(plan[5,i]),
                'th1': str(joint_pos[0,i]), 'th2': str(joint_pos[1,i]), 'd3': str(joint_pos[2,i]),
                'th1_dot': str(joint_vel[0,i]), 'th2_dot': str(joint_vel[1,i]), 'd3_dot': str(joint_vel[2,i]), 'measured_th1': str(measured_joint_pos[0,i]),'measured_th2': str(measured_joint_pos[1,i]),'measured_d3': str(measured_joint_pos[2,i]),
                'measured_th1_dot': str(measured_joint_vel[0,i]),'measured_th2_dot': str(measured_joint_eff[1,i]),'measured_d3_dot': str(measured_joint_vel[2,i]),
                'eff_1': str(measured_joint_eff[0,i]), 'eff_2': str(measured_joint_eff[1,i]), 'eff_3': str(measured_joint_eff[2,i]), 'time': str(time[0,i])})
            else:
                writer.writerow({'x': str(plan[0,i]), 'y': str(plan[1,i]), 'z': str(plan[2,i]), 'x_dot': str(plan[3,i]), 'y_dot': str(plan[4,i]), 'z_dot': str(plan[5,i]),
                'th1': str(joint_pos[0,i]), 'th2': str(joint_pos[1,i]), 'd3': str(joint_pos[2,i]),
                'th1_dot': str(joint_vel[0,i]), 'th2_dot': str(joint_vel[1,i]), 'd3_dot': str(joint_vel[2,i])})

            rate.sleep()

print('Export Complete.')
print('--------------------------------------')

while not rospy.is_shutdown():
    pass

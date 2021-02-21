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
from matplotlib import pyplot as plt

state = np.zeros(13)
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/dynamic_output.csv'
exe_rate = 50

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
        state[6] = float(msg.acceleration[0])
        state[7] = float(msg.acceleration[1])
        state[8] = float(msg.acceleration[2])
        state[9] = float(msg.effort[0])
        state[10] = float(msg.effort[1])
        state[11] = float(msg.effort[2])
        state[12] = time.to_sec()

    except:
        pass

rospy.init_node('effort_trajectory')
rate = rospy.Rate(exe_rate)

pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#subscribe to joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('----------Effort Trajectory-----------')

x1 = 0.134 #input('Initial X?')
x2 = 0.16 #input('Final X?')
y1 = 0.02 #input('Initial Y?')
y2 = -0.02 #input('Final Y?')
z1 = -0.05008 #input('Initial Z?')
z2 = 0.01 #input('Final Z? ')

print('--------------------------------------')

initial_pos = np.array([[x1], [y1], [z1]], dtype="float")
final_pos = np.array([[x2], [y2], [z2]], dtype="float")

T = input('Input Time to Complete Trajectory (s): ')
N = input('Input Number of Points along Trajectory: ')

plan, dt = robot.trajectory_plan(initial_pos, final_pos, T, N)

print('--------------------------------------')
print('Moving to start position..')

#IK to move manipulator to starting pose
target = initial_pos
error_signal = np.array([[1],[1],[1]])
last_error_signal = error_signal
k = 1.0
kp = 15.0
kd = 1.0

#viscoelastic force control loop
while np.any(error_signal > 1e-5):
    robot.th1 = state[0]
    robot.th2 = state[1]
    robot.d3 = state[2]
    robot.get_fk()

    print('--------------------------------------')
    current_pose = np.array([[robot.x],[robot.y],[robot.z]], dtype=float).reshape(3,1)
    print('Current Pose: ' + str(robot.x) + ' ' + str(robot.y) + ' ' + str(robot.z))
    error_signal = k * ( target - current_pose )
    print('Target: ' + str(x1) + ' ' + str(y1) + ' ' + str(z1))
    diff_error_signal = exe_rate * ( error_signal - last_error_signal )
    print('Error Signal: ' + str(error_signal[0]) + ' ' + str(error_signal[1]) + ' ' + str(error_signal[2]))
    force = kp * error_signal + kd * diff_error_signal + robot.get_G()
    print('Force: ' + str(force[0]) + ' ' + str(force[1]) + ' ' + str(force[2]))
    efforts = np.matmul(np.transpose(robot.get_Jv()),force)
    print('Efforts: ' + str(efforts[0]) + ' ' + str(efforts[1]) + ' ' + str(efforts[2]))

    pub1.publish(-1*efforts[0])
    pub2.publish(-1*efforts[1])
    pub3.publish(efforts[2])

    last_error_signal = error_signal

    rate.sleep()

joint_pos = np.zeros((3,N))
joint_vel = np.zeros((3,N))
joint_accel = np.zeros((3,N))
joint_effort = np.zeros((3,N))

print('Planning Trajectory..')
time = np.linspace(0,T,N)

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

    robot.x_ddot = plan[6,i]
    robot.y_ddot = plan[7,i]
    robot.z_ddot = plan[8,i]
    robot.inv_accel_kin()

    #store joint space trajectory data
    joint_pos[0,i] = -1*robot.th1
    joint_pos[1,i] = robot.th2
    joint_pos[2,i] = robot.d3

    joint_vel[0,i] = -1*robot.th1_dot
    joint_vel[1,i] = robot.th2_dot
    joint_vel[2,i] = robot.d3_dot

    joint_accel[0,i] = -1*robot.th1_ddot
    joint_accel[1,i] = robot.th2_ddot
    joint_accel[2,i] = robot.d3_ddot

    robot.get_efforts()
    joint_effort[0,i] = -1*robot.th1_eff
    joint_effort[1,i] = robot.th2_eff
    joint_effort[2,i] = robot.d3_eff

    rate.sleep()

plt1.subplot(2,2,1)
plt.plot(time,joint_effprt[0,:].reshape(1,N))
plt.title('Pitch Effort')
plt1.subplot(2,2,2)
plt.plot(time,joint_effort[1,:].reshape(1,N))
plt.title('Yaw Effort')
plt1.subplot(2,2,3)
plt.plot(time,joint_effort[2,:].reshape(1,N))
plt.title('Ext. Effort')
plot.show()

print('Executing Trajectory..')
print(math.floor(1/dt))
rate = rospy.Rate(math.floor(1/dt))

measured_joint_eff = np.zeros((3,N))
measured_joint_pos = np.zeros((3,N))

start = rospy.get_time()

for i in range(0,N):
    #set joint effort at waypoint i
    pub1.publish(-1*joint_effort[0,i])
    pub2.publish(-1*joint_effort[1,i])
    pub3.publish(joint_effort[2,i])

    #record joint variables from joint_state_publisher
    measured_joint_pos[0,i] = state[0]
    measured_joint_pos[1,i] = state[1]
    measured_joint_pos[2,i] = state[2]
    measured_joint_eff[0,i] = state[9]
    measured_joint_eff[1,i] = state[10]
    measured_joint_eff[2,i] = state[11]

    rate.sleep()

print('--------------------------------------')

rate = rospy.Rate(exe_rate)

time_elapsed = rospy.get_time() - start
print('Total Time: ' + str(time_elapsed))

print(' ')
print('Exporting data to csv..')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    data = ['eff_1','eff_2','eff_3','th1','th2','d3','measured_eff_1','measured_eff_2','measured_eff_3','measured_th1','measured_th2','measured_d3','time']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    for i in range(0,N):
        writer.writerow({'eff_1': str(joint_effort[0,i]), 'eff_2': str(joint_eff[1,i]), 'eff_3': str(joint_eff[2,i]), 'th1': str(plan[0,i]), 'th2': str(plan[1,i]), 'd3': str(plan[2,i]), 'measured_eff_1': str(measured_joint_eff[0,i]), 'measured_eff_2': str(measured_joint_eff[1,i]), 'measured_eff_3': str(measured_joint_eff[2,i]), 'calculated_eff_1': str(calculated_joint_eff[0,i]),
        'calculated_eff_1': str(calculated_joint_eff[1,i]), 'calculated_eff_1': str(calculated_joint_eff[2,i]), 'time': str(time[0,i])})

        rate.sleep()

print('Export Complete.')
print('--------------------------------------')

while not rospy.is_shutdown():
    pass

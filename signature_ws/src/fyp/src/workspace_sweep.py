#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64

state = np.zeros(4)
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/workspace.csv'
exe_rate = 12

def joint_reader(msg):
    global state

    time = rospy.Duration()
    time = msg.header.stamp

    try:
        #joints published alphabetically (ext, pitch, yaw)
        state[0] = round(float(msg.position[1]),6)
        state[1] = round(float(msg.position[2]),6)
        state[2] = round(float(msg.position[0]),6)

        state[3] = time.to_sec()
    except:
        print('Joint State Error')

#float publishers to arm_controller/effort/joint/command to set joint efforts
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=10)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=10)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=10)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

rospy.init_node('workspace_sweep')

print('----------Workspace Sweep-------------')
print('--------------------------------------')

robot = signaturebot.signature_bot()
rate = rospy.Rate(exe_rate)

time.sleep(4)

#joint space locations
th1_range = np.linspace(-1.57, 0.38, 30, endpoint=True)
th2_range_1 = np.linspace(0, -1.57, 30, endpoint=True)
th2_range_2 = np.linspace(0, 1.57, 30, endpoint=True)
d3_range = np.linspace(0, 0.096, 16, endpoint=True)

N = 10000

ws = np.zeros((3,N))
count = 0

for i in th1_range:
    pitch_pub.publish(i)

    for k in d3_range:
        ext_pub.publish(k)

        for j in th2_range_1:
            yaw_pub.publish(j)

            if j == 0.0:
                time.sleep(0.5)

            rate.sleep()

            robot.th1 = state[0]
            robot.th2 = state[1]
            robot.d3 = state[2]
            robot.get_fk()

            yaw_error = abs(j - robot.th2)
            #if off by 1 degree -> exit loop
            if (yaw_error > 0.02):
                #store collision boundary
                ws[0,count] = robot.x
                ws[1,count] = robot.y
                ws[2,count] = robot.z
                count += 1

                break

            if k == 0.0 or k == 0.096 or i == -1.57 or i == 0.38:
                #store boundary
                ws[0,count] = robot.x
                ws[1,count] = robot.y
                ws[2,count] = robot.z
                count += 1

            print('Data Count: ' + str(count) + '/' + str(N))
            sys.stdout.write("\033[F")
            sys.stdout.write("\033[K")

        for j in th2_range_2:
            yaw_pub.publish(j)

            if j == 0.0:
                time.sleep(0.5)

            rate.sleep()

            robot.th1 = state[0]
            robot.th2 = state[1]
            robot.d3 = state[2]
            robot.get_fk()

            yaw_error = abs(j - robot.th2)
            #if off by 1 degree -> exit loop
            if (yaw_error > 0.02):
                #store collision boundary
                ws[0,count] = robot.x
                ws[1,count] = robot.y
                ws[2,count] = robot.z
                count += 1

                break

            if k == 0.0 or k == 0.096 or i == -1.57 or i == 0.38:
                #store boundary
                ws[0,count] = robot.x
                ws[1,count] = robot.y
                ws[2,count] = robot.z
                count += 1

            print('Data Count: ' + str(count) + '/' + str(N))
            sys.stdout.write("\033[F")
            sys.stdout.write("\033[K")

print('--------------------------------------')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    data = ['x','y','z']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    for i in range(0,N):
        if ws[0,i] != 0.0 or ws[1,i] != 0.0 or ws[2,i] != 0.0:
            writer.writerow({'x': str(ws[0,i]), 'y': str(ws[1,i]), 'z': str(ws[2,i])})

        rate.sleep()

print('Export Complete.')
print('--------------------------------------')


while not rospy.is_shutdown():
    rate.sleep()

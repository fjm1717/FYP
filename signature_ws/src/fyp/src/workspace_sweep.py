#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64

state = np.zeros(4)
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/workspace.csv'
exe_rate = 2

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

time.sleep(3)

#joint space locations
th1_range = np.linspace(-1.57, 0.65, 12, endpoint=True)
th2_range = np.linspace(-1.57, 1.57, 20, endpoint=True)
d3_range = np.linspace(0, 0.096, 2, endpoint=True)

N = 12*20*2

ws = np.zeros((3,N))
count = 0

for i in th1_range:
    for j in th2_range:
        for k in d3_range:

            pitch_pub.publish(i)
            yaw_pub.publish(j)
            ext_pub.publish(k)

            robot.th1 = state[0]
            robot.th2 = state[1]
            robot.d3 = state[2]
            robot.get_fk()

            rate.sleep()

            ws[0,count] = robot.x
            ws[1,count] = robot.y
            ws[2,count] = robot.z

            print('EE Position (mm): ' + str(robot.x*1.0e3) + ' ' + str(robot.y*1.0e3) + ' ' + str(robot.z*1.0e3))
            sys.stdout.write("\033[F")
            sys.stdout.write("\033[K")

            count += 1

print('--------------------------------------')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    data = ['x','y','z']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    for i in range(0,N):
        writer.writerow({'x': str(ws[0,i]), 'y': str(ws[1,i]), 'z': str(ws[2,i])})

    rate.sleep()

print('Export Complete.')
print('--------------------------------------')


while not rospy.is_shutdown():
    rate.sleep()

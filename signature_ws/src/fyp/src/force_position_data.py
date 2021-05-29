#!/usr/bin/python2.7

import rospy
import time
import csv
import os
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt

output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/data/force_position/force_position_data.csv'

state = np.zeros(7)
N = 400
rms_error = 1.0
exe_rate = 50

centre = np.array([[0.165],[0.0],[-0.05008]])

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        #joints published alphabetically (ext, pitch, yaw)
        time = msg.header.stamp
        state[0] = round(float(msg.position[1]),4)
        state[1] = round(float(msg.position[2]),4)
        state[2] = round(float(msg.position[0]),4)
        state[3] = time.to_sec()
        state[4] = round(float(msg.effort[1]),4)
        state[5] = round(float(msg.effort[2]),4)
        state[6] = round(float(msg.effort[0]),4)
    except:
        pass

def error_reader(msg):
    global rms_error

    try:
        rms_error = msg.data
    except:
        pass

rospy.init_node('force_position_data')
rate = rospy.Rate(exe_rate)

#float publishers to arm_controller/effort/joint/command to set joint efforts
eff_pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
eff_pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
eff_pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#publiser to set PID force_position control and recieve rms error
position_pub = rospy.Publisher('signaturebot/force_position/command', Twist, queue_size=10)
error_sub = rospy.Subscriber('signaturebot/force_position/error', Float64, error_reader)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('----------Force Position Data----------')

time.sleep(6)

print('---------------------------------------')
print('Moving EE to Target..')

command = Twist()
command.linear.x = centre[0]
command.linear.y = centre[1]
command.linear.z = centre[2]

data = np.zeros((4,N))
count = 0

position_pub.publish(command)
while count < N:

    #store current location of robot in joint and task space
    robot.th1 = state[0]
    robot.th2 = state[1]
    robot.d3 = state[2]
    robot.get_fk()

    data[0,count] = robot.x - centre[0]
    data[1,count] = robot.y - centre[1]
    data[2,count] = robot.z - centre[2]
    data[3,count] = state[3]

    print('Data Points: ' + str(count) + '/' + str(N) + ' Efforts: ' + str(state[4]) + ' ' + str(state[5]) + ' ' + str(state[6]))
    sys.stdout.write("\033[F")
    sys.stdout.write("\033[K")

    count += 1

    rate.sleep()

#switch off controller
command.linear.x = 0.0
command.linear.y = 0.0
command.linear.z = 0.0
position_pub.publish(command)

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    titles = ['x','y','z','time']
    writer = csv.DictWriter(csv_file, fieldnames=titles)
    writer.writeheader()

    for i in range(0,N):
        writer.writerow({'x': str(data[0,i]), 'y': str(data[1,i]), 'z': str(data[2,i]), 'time': str(data[3,i] - data[3,0])})

        rate.sleep()

print('Export Complete..')

plt.plot(data[3,:].reshape(N,1),data[0,:].reshape(N,1),data[3,:].reshape(N,1),data[1,:].reshape(N,1),data[3,:].reshape(N,1),data[2,:].reshape(N,1))
plt.ylabel('Error /m')
plt.xlabel('Time /secs')
plt.legend(['x','y','z'])

plt.show()


while not rospy.is_shutdown():
    pass

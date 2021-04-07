#!/usr/bin/python2.7

import rospy
import time
import copy
import csv
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64
import matplotlib.pyplot as plt

state = np.zeros(4)
sample_rate = 200
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/horizontal_500.csv'

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        #joints published in urdf order (pitch, yaw, ext)
        time = msg.header.stamp
        state[0] = round(float(msg.position[0]),6)
        state[1] = round(float(msg.position[1]),6)
        state[2] = round(float(msg.position[2]),6)

        state[3] = time.to_sec()
    except:
        pass

rospy.init_node('effort_position')

#float publishers to arm_controller/effort/joint/command to set joint efforts
pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

robot.th1 = 0.0
robot.th2 = 0.0
N = 2000

#motor parameters
n = 30.0/14.0
stall_torque = 49.4 #mNm
nom_torque = 12.5 #mNm
no_load_curr = 28.3 #mA
start_curr = 3140 #mA
i = 500.0 #mA

measurements = np.zeros(N)

#encoder and pinion data
res = 2*math.pi / 20000 #rads/count
r = 6.25e-3 #mm

print('----------Extension Data Collection----------')
rate = rospy.Rate(sample_rate)

time.sleep(3)

pitch_pub.publish(robot.th1)
yaw_pub.publish(robot.th2)
ext_pub.publish(-10.0)

T3 = nom_torque + ( ( stall_torque - nom_torque ) / ( start_curr - no_load_curr ) ) * ( i - no_load_curr ) #mNm
F3 = n * T3 / (r*1.0e3) #N

print('---------------------------------------------')
print('Motor Torque: ' + str(T3) + 'mNm')
print('Output Force: ' + str(F3) + 'N')

time.sleep(1)

count = 0
robot.d3 = 0
enc_pos = 0

ext_pub.publish(F3)

while robot.d3 <= 50.0e-3 and count < N:

    robot.d3 = state[2]
    enc_pos = math.floor( robot.d3 / (res*r) )
    measurements[count] = enc_pos
    count += 1

    rate.sleep()

print('50mm Extension Reached!')
print(str(count) + ' Data Points!')
print('---------------------------------------------')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    data = ['encoder_position']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    for i in range(0,N):
        if measurements[i] != 0.0:
            writer.writerow({'encoder_position': str(measurements[i])})

    rate.sleep()

print('Export Complete.')

encoder_plot = measurements[measurements != 0]

plt.plot(encoder_plot)
plt.title('Encoder Readings')
plt.show()

while not rospy.is_shutdown():
    pass

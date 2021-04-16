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
import matplotlib.pyplot as plt

state = np.zeros(4)
sample_rate = 10
output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/linear_data/horizontal/horiz_1000.csv'
input_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/trajectory_output/linear_data/horizontal/poly_horiz_1000.csv'

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
N = 600 #number of data points
M = 5 #number of spring constants

#motor parameters
n = 30.0/14.0
stall_torque = 49.4 #mNm
nom_torque = 12.5 #mNm
no_load_curr = 28.3 #mA
start_curr = 3140 #mA
i = 1000.0 #mA

#extended model
spring_constants = np.linspace(40.0,45.0,M) #N/m

measurements = np.zeros((2*M,N))

#encoder and pinion data
res = 2*math.pi / 20000 #rads/count
r = 6.25e-3 #m

print('----------Extension Data Collection----------')
rate = rospy.Rate(sample_rate)

time.sleep(3)
pitch_pub.publish(robot.th1)
yaw_pub.publish(robot.th2)

T3 = nom_torque + ( ( stall_torque - nom_torque ) / ( start_curr - no_load_curr ) ) * ( i - no_load_curr ) #mNm
F3 = n * T3 / (r*1.0e3) #N

print('---------------------------------------------')
print('Motor Torque: ' + str(T3) + 'mNm')
print('Output Force: ' + str(F3) + 'N')
print('---------------------------------------------')

measurement_count = 0

for k in spring_constants:

    print('---------------------------------------------')
    print('Spring Constant: ' + str(k) + 'N/m')

    ext_pub.publish(-10.0)

    time.sleep(3)

    count = 0
    robot.d3 = 0
    enc_pos = 0

    while robot.d3 <= 50.0e-3 and count < N:

        robot.d3 = state[2]

        F_effective = F3 - k*robot.d3
        ext_pub.publish(F_effective)

        enc_pos = math.floor( robot.d3 / (res*r) )
        measurements[2*measurement_count,count] = enc_pos
        measurements[2*measurement_count+1,count] = state[3]
        count += 1

        print('F_effective: ' + str(F_effective) + 'N')
        sys.stdout.write("\033[F")
        sys.stdout.write("\033[K")

        rate.sleep()

    print('---------------------------------------------')
    print('50mm Extension Reached!')

    measurement_count += 1

print('---------------------------------------------')
print('Export Complete.')

#open csv to retrieve row sum
with open(input_path) as csv_file:
    reader = csv.reader(csv_file)
    row_sum = sum(1 for row in reader)

row_count = 0
exp_results = np.zeros((2,row_sum))

#read in actual data
with open(input_path) as csv_file:
    reader = csv.reader(csv_file)

    for row in reader:
        exp_results[0,row_count] = row[0]
        exp_results[1,row_count] = row[1]

        row_count += 1

plt.plot(exp_results[1,:].reshape(row_sum,1),exp_results[0,:].reshape(row_sum,1))
plt.title('Results')
plt.xlabel('Time (s)')
plt.ylabel('Encoder Count')

#export data to output.csv
with open(output_path, mode='w') as csv_file:
    mywriter = csv.writer(csv_file, delimiter=',')

    for k in range(0,M):
        encoder_values = measurements[2*k,:].reshape(N,1)
        time = measurements[2*k+1,:].reshape(N,1)

        y = encoder_values[encoder_values != 0]
        Z = int(y.size)
        y = y.reshape(Z,1)
        x = np.zeros((Z,1))
        for i in range(0,Z):
            x[i] = time[i] - time[0]

        mywriter.writerow(x[:,0])
        mywriter.writerow(y[:,0])

        #add measured values to plot
        plt.plot(x,y)

plt.legend(['Experimental'])
plt.show()

while not rospy.is_shutdown():
    pass

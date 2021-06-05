#!/usr/bin/python2.7

import rospy
import time
import csv
import os
import sys
import math
import signaturebot
import numpy as np
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Header, Float64
from geometry_msgs.msg import Twist, WrenchStamped, Wrench, Vector3, PointStamped, Point

plane = 'xz'

if plane == 'xy':
    output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/data/planar_constraint_data_xy.csv'
elif plane == 'xz':
    output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/data/planar_constraint_data_xz.csv'
elif plane == 'yz':
    output_path = '/home/spyros/Spyros/FYP/signature_ws/src/fyp/data/planar_constraint_data_yz.csv'
else:
    pass

state = np.zeros(4)
xbox = np.zeros(6)
rms_error = 1.0
exe_rate = 60

#spherical constraints
centre = np.array([[0.175],[0.0],[-0.05008]])
radius = 0.025

def xbox_reader(msg):
    global xbox
    xbox[0] = float(msg.axes[4]) #z-axis
    xbox[1] = float(msg.axes[1]) #x-axis
    xbox[2] = float(msg.axes[0]) #y-axis
    xbox[3] = int(msg.buttons[0]) #A
    xbox[4] = int(msg.buttons[1]) #B
    xbox[5] = int(msg.buttons[2]) #X

def joint_reader(msg):
    global state

    time = rospy.Duration()
    try:
        #joints published alphabetically (ext, pitch, yaw)
        time = msg.header.stamp
        state[0] = float(msg.position[1])
        state[1] = float(msg.position[2])
        state[2] = float(msg.position[0])
        state[3] = time.to_sec()
    except:
        pass

def error_reader(msg):
    global rms_error

    try:
        rms_error = msg.data
    except:
        pass

rospy.init_node('active_constraint_demo')
rate = rospy.Rate(exe_rate)

#subscribe to /joy to recieve controller inputs
joy_sub = rospy.Subscriber('joy', Joy, xbox_reader)

#float publishers to arm_controller/effort/joint/command to set joint efforts
eff_pub1 = rospy.Publisher('signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
eff_pub2 = rospy.Publisher('signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
eff_pub3 = rospy.Publisher('signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)

#publiser to set PID force_position control and recieve rms error
position_pub = rospy.Publisher('signaturebot/force_position/command', Twist, queue_size=10)
error_sub = rospy.Subscriber('signaturebot/force_position/error', Float64, error_reader)

#publish force vector at end effector to visualise in rviz
wrench_pub = rospy.Publisher('signaturebot/wrench', WrenchStamped, queue_size=10)

#publish spherical boundart in world frame to visualise in rviz
sphere_pub = rospy.Publisher('signaturebot/sphere', PointStamped, queue_size=10)

#subscribe to /joint_state to monitor joint position, velocities etc.
joint_sub = rospy.Subscriber('signaturebot/joint_states', JointState, joint_reader)

robot = signaturebot.signature_bot()

print('----------Planar Constraint Demo----------')

#user force gains
if plane == 'xy':
    kf = np.diag([1.0,1.0,0.0])
elif plane == 'xz':
    kf = np.diag([1.0,0.0,1.0])
elif plane == 'yz':
    kf = np.diag([0.0,1.0,1.0])
else:
    kf = np.diag([1.0,1.0,1.0])

Q = np.diag([1.0, 1.0, 4.8])
#viscoelastic force gains
ke = np.diag([1870.0,1720.0,1660.0])
kv = np.diag([2.0,2.0,1.8])
#planar constraint
k = 200

#motor parameters
n = 30.0/14.0
stall_torque = 49.4 #mNm
nom_torque = 0 #12.5 #mNm
no_load_curr = 28.3 #mA
start_curr = 3140 #mA
r = 6.25e-3 #m

pose = np.zeros((3,1))
dx = np.zeros((3,1))
last_dx = np.zeros((3,1))
force = np.zeros((3,1))
efforts = np.zeros((3,1))

time.sleep(4)

print('----------------------------------------')
print('Moving EE to Boundary Centre..')

command = Twist()
command.linear.x = centre[0]
command.linear.y = centre[1]
command.linear.z = centre[2]

position_pub.publish(command)
while(rms_error > 5.0e-3):
    pass

print('Switched to User Input..')

command.linear.x = 0.0
command.linear.y = 0.0
command.linear.z = 0.0
#turn off position controller
position_pub.publish(command)

#set up wrench msg
wrench_msg = WrenchStamped()
wrench_msg.header.frame_id = "extension_link"
wrench_msg.wrench.torque.x = 0.0
wrench_msg.wrench.torque.y = 0.0
wrench_msg.wrench.torque.z = 0.0

#publish /point boundary
sphere = PointStamped()
sphere.header.frame_id = "world"
sphere.header.stamp = rospy.get_rostime()
#world to base rotation matrix
R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
world_centre = np.matmul(R,centre)
sphere.point.x = world_centre[0] + robot.px
sphere.point.y = world_centre[1] + robot.py
sphere.point.z = world_centre[2] + robot.pz
sphere_pub.publish(sphere)

print('----------------------------------------')

start_time = rospy.get_time()
dist = 0

with open(output_path, mode='w') as csv_file:
    data = ['p','q','force','dev','ext_curr','pitch','yaw','ext']
    writer = csv.DictWriter(csv_file, fieldnames=data)
    writer.writeheader()

    #cancel loop by pressing A
    while (xbox[3]==0):

        #store current location of robot in joint and task space
        robot.th1 = state[0]
        robot.th2 = state[1]
        robot.d3 = state[2]
        robot.get_fk()

        pose[0] = robot.x
        pose[1] = robot.y
        pose[2] = robot.z

        if plane == 'xy':
            p = robot.x
            q = robot.y
        elif plane == 'xz':
            p = robot.x
            q = robot.z
        elif plane == 'yz':
            p = robot.y
            q = robot.z
        else:
            p = 0
            q = 0

        #end-effector force from user input
        user_input = np.array([[xbox[1]],[xbox[2]],[xbox[0]]])
        force = np.matmul(kf,user_input)
        G = robot.get_G()

        #print('Grav. Efforts: ' + str(G[0]) + ' ' + str(G[1]) + ' ' + str(G[2]))
        #sys.stdout.write("\033[F")
        #sys.stdout.write("\033[K")

        #active constraint
        planar_force = np.zeros((3,1))
        viscoelastic_force = np.zeros((3,1))
        if ( ( pow(robot.x - centre[0],2) + pow(robot.y - centre[1],2) + pow(robot.z - centre[2],2) ) >= pow(radius,2) ):
            #outside boundary
            dist = np.linalg.norm(centre-pose)
            dx = ( centre - pose ) * ( 1 - ( radius / dist ) )
            dx_dot = exe_rate * ( dx - last_dx )
            viscoelastic_force = np.matmul(ke,dx) + np.matmul(kv,dx_dot)
            dist = dist - radius
        else:
            dist = 0

        #publish wrench msg
        wrench_msg.header.stamp = rospy.get_rostime()
        #transformed from world to end effector ref frame
        ee_force = np.matmul(robot.get_invR(),viscoelastic_force)
        wrench_msg.wrench.force.x = ee_force[0]
        wrench_msg.wrench.force.y = ee_force[1]
        wrench_msg.wrench.force.z = ee_force[2]
        wrench_pub.publish(wrench_msg)

        #republish boundary for fun
        sphere.header.stamp = rospy.get_rostime()
        sphere_pub.publish(sphere)

        if plane == 'xy':
            fz = -k*( robot.z - np.asscalar(centre[2]) )
            planar_force = np.array([[0],[0],[fz]])
        elif plane == 'xz':
            fy = -k*( robot.y - np.asscalar(centre[1]) )
            planar_force = np.array([[0],[fy],[0]])
        elif plane == 'yz':
            fx = -k*( robot.x - np.asscalar(centre[0]) )
            planar_force = np.array([[fx],[0],[0]])
        else:
            pass

        viscoelastic_efforts = np.matmul(np.transpose(robot.get_Jv()),viscoelastic_force)
        planar_efforts = np.matmul(np.transpose(robot.get_Jv()),planar_force)
        user_effort = np.matmul(np.transpose(robot.get_Jv()),force)
        scaled_visco = np.matmul(Q,viscoelastic_efforts)
        scaled_user = np.matmul(Q,user_effort)
        efforts = np.matmul(Q,planar_efforts) + G + scaled_visco + scaled_user

        norm_force = np.linalg.norm(viscoelastic_force)

        tau1 = np.asscalar(scaled_visco[0])
        tau2 = np.asscalar(scaled_visco[1])
        F3 = np.asscalar(scaled_visco[2])

        #extension motor current
        T3 = F3 * (r*1.0e3) / n #mNm
        i = ( abs(T3) - nom_torque ) * ( ( start_curr - no_load_curr ) / ( stall_torque - nom_torque ) ) + no_load_curr #mA

        #publish efforts to gazebo
        eff_pub1.publish(efforts[0])
        eff_pub2.publish(efforts[1])
        eff_pub3.publish(efforts[2])

        #print('Extension Motor Current (mA): ' + str(i))
        #print('Extension Force (N): ' + str(scaled_user[2]))
        print('Boundary Penetration (mm): ' + str(dist*1000))

        if dist*1000 < 1.0:
            sys.stdout.write("\033[F")
            sys.stdout.write("\033[K")

        #upload state to csv
        if dist != 0:
            writer.writerow({'p': str(p), 'q': str(q), 'force': str(norm_force), 'dev': str(dist), 'ext_curr': str(i), 'pitch': str(tau1), 'yaw': str(tau2), 'ext': str(F3)})

        rate.sleep()

print('Export Complete..')

while not rospy.is_shutdown():
    pass

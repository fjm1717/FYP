#!/usr/bin/python2.7

import rospy
import math
import kinematics
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

z_dot = 0.0
x_dot = 0.0
y_dot = 0.0
zero_state = 0

def callback(msg):
    global z_dot
    global x_dot
    global y_dot
    global zero_state
    z_dot = float(msg.axes[4])
    x_dot = -1*float(msg.axes[1])
    y_dot = -1*float(msg.axes[0])
    zero_state = int(msg.buttons[0])

rospy.init_node('joy_reader')

joy_sub = rospy.Subscriber('joy',Joy,callback)

pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008)

print('-----------------------------')
print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
print('-----------------------------')
print('th1: ' + str(robot.th1) + ' th2: ' + str(robot.th2) + ' d3: ' + str(robot.d3))
print('-----------------------------')

rate = rospy.Rate(50)

while not rospy.is_shutdown():

    if zero_state == 1:
        robot.th1 = 0.0
        robot.th2 = 0.0
        robot.d3 = 0.0
        robot.get_fk()

    robot.z = robot.z + z_dot * 0.0005
    robot.x = robot.x - x_dot * 0.0005
    robot.y = robot.y + y_dot * 0.0005

    robot.get_ik()

    if robot.th1 > 1.57:
        robot.th1 = 1.57
    elif robot.th1 < -1.57:
        robot.th1 = -1.57
    elif robot.th2 > 1.57:
        robot.th2 = 1.57
    elif robot.th2 < -1.57:
        robot.th2 = -1.57
    elif robot.d3 > 0.06:
        robot.d3 = 0.06
    elif robot.d3 < 0:
        robot.d3 = 0

    robot.get_fk()

    pitch_pub.publish(robot.th1)
    yaw_pub.publish(robot.th2)
    ext_pub.publish(robot.d3)

    print('-----------------------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('-----------------------------')
    print('th1: ' + str(robot.th1) + ' th2: ' + str(robot.th2) + ' d3: ' + str(robot.d3))
    print('-----------------------------')

    rate.sleep()

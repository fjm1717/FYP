#!/usr/bin/python2.7

import rospy
import signaturebot
from std_msgs.msg import Float64

pub1 = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)
rospy.init_node('position_command')

rate = rospy.Rate(10)

robot = signaturebot.signature_bot()

robot.th1 = 0.0
robot.th2 = 0.0
robot.d3 = 0.0
robot.get_fk()

rate.sleep()

pub1.publish(-1*robot.th1)
pub2.publish(-1*robot.th2)
pub3.publish(robot.d3)

while not rospy.is_shutdown():

    print('')
    print('Current Position: ')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('pitch: ' + str(robot.th1) + ' yaw: ' + str(robot.th2) + ' ext: ' + str(robot.d3))
    print('')

    robot.x = input('Set X: ')
    robot.y = input('Set Y: ')
    robot.z = input('Set Z: ')
    robot.get_ik()

    pub1.publish(-1*robot.th1)
    pub2.publish(-1*robot.th2)
    pub3.publish(robot.d3)

    rate.sleep()

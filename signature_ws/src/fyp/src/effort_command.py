#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Float64

pub1 = rospy.Publisher('/signaturebot/arm_controller/effort/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('/signaturebot/arm_controller/effort/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('/signaturebot/arm_controller/effort/extension_joint/command', Float64, queue_size=1)
rospy.init_node('effort_command')

rate = rospy.Rate(20)

pitch = 0.0
yaw = 0.0
extension = 0.0

rate.sleep()

pub1.publish(pitch)
pub2.publish(yaw)
pub3.publish(extension)

while not rospy.is_shutdown():

    print('')
    print('Current Efforts: ')
    print('Pitch: ' + str(pitch) + ' Yaw: ' + str(yaw) + ' Extension: ' + str(extension))
    print('')

    pitch = input('Set Pitch: ')
    yaw = input('Set Yaw: ')
    extension = input('Set Extension: ')

    pub1.publish(pitch)
    pub2.publish(yaw)
    pub3.publish(extension)

    rate.sleep()

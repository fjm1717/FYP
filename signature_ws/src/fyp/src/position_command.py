#!/usr/bin/python2.7

import rospy
from std_msgs.msg import Float64

pub1 = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
pub2 = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
pub3 = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)
rospy.init_node('position_command')

rate = rospy.Rate(10)

pitch = 0.0
yaw = 0.0
extension = 0.0

rate.sleep()

pub1.publish(pitch*0.0174533)
pub2.publish(yaw*0.0174533)
pub3.publish(extension)

while not rospy.is_shutdown():

    print('')
    print('Current Position: ')
    print('Pitch: ' + str(pitch) + ' Yaw: ' + str(yaw) + ' Extension: ' + str(extension))
    print('')

    pitch = input('Set Pitch: ')
    yaw = input('Set Yaw: ')
    extension = input('Set Extension: ')

    pub1.publish(pitch*0.0174533)
    pub2.publish(yaw*0.0174533)
    pub3.publish(extension)

    rate.sleep()

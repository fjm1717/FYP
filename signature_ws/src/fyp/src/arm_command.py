#!/usr/bin/python2.7

import rospy
import numpy
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size=1)
rospy.init_node('arm_command')

rate = rospy.Rate(10)

pitch = 0.0
yaw = 0.0
extension = 0.0

output = JointTrajectory()
point = JointTrajectoryPoint()
point.time_from_start = rospy.Duration(2)
output.points.append(point)

output.header = Header()
output.joint_names = ['joint_p_1', 'joint_y_1', 'joint_s_1']

output.header.stamp = rospy.Time.now()
output.points[0].positions = [0, 0, 0, 0]

pub.publish(output)

while not rospy.is_shutdown():

    print('')
    print('Current State: ')
    print('Pitch: ' + str(pitch) + ' Yaw: ' + str(yaw) + ' Extension: ' + str(extension))
    print('')

    pitch = input('Set Pitch: ')
    yaw = input('Set Yaw: ')
    extension = input('Set Extension: ')

    output.header.stamp = rospy.Time.now()
    output.points[0].positions = [(pitch*0.0174533), (yaw*0.0174533), (extension*0.0174533)]

    pub.publish(output)

    rate.sleep()

#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("signature_group")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()
print('Current Values: ' + group_variable_values)

group_variable_values[0] = -1*(45*0.0174533)
group_variable_values[1] = 25*0.0174533
group_variable_values[2] = 0.075
group.set_joint_value_target(group_variable_values)

plan2 = group.plan()
group.go(wait=True)

rospy.sleep(5)

moveit_commander.roscpp_shutdown()

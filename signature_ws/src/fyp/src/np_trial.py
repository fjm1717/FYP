#!/usr/bin/python2.7

import rospy
import math
import kinematics
import numpy as np
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Header, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node('Trial')

robot = kinematics.signature_bot(0,0,0,0,0,0,0,0,0,1,0.5,0.2)
robot.inv_vel_kin()
print(robot.th1_dot)

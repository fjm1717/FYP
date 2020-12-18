#!/usr/bin/python2.7

import kinematics
import numpy as np

robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008, 0, 0, 0, 0.1, 0.2, -0.1)

joint_dot = np.matmul(robot.get_invJv(),np.array([[robot.x_dot],[robot.y_dot],[robot.z_dot]]))
print(joint_dot)

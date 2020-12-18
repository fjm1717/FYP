#!/usr/bin/python2.7

import kinematics

robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008)
J = robot.get_invJv()

print(J)

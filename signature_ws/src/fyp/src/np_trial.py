#!/usr/bin/python2.7

import kinematics
import numpy as np

robot = kinematics.signature_bot(0.134, 0.05008, 0, 0, 0, 0.134, 0, -0.05008, 0, 0, 0, 0, 0, 0)
initial_pos = np.array([0,1,0])
final_pos = np.array([12,50,-20])
plan, dt = robot.traj_plan(initial_pos, final_pos, 2, 50)

print(plan)

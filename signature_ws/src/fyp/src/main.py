# coding=utf-8
# This is a sample Python script.

import kinematics as kin

if __name__ == '__main__':
    robot = kin.signature_bot(0.134, 0.05008, 0, 0, 0, 0, 0, 0)

    robot.get_fk()
    print('-----------------------------')
    print('x: ' + str(robot.x) + ' y: ' + str(robot.y) + ' z: ' + str(robot.z))
    print('-----------------------------')
    robot.get_ik()
    print('th1: ' + str(robot.th1) + ' th2: ' + str(robot.th2) + ' d3: ' + str(robot.d3))
    print('-----------------------------')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/

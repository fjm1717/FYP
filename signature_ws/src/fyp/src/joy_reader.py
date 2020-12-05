#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

ext_speed = 0.0
pitch_speed = 0.0
yaw_speed = 0.0
zero_state = 0

def callback(msg):
    global ext_speed
    global pitch_speed
    global yaw_speed
    global zero_state
    ext_speed = float(msg.axes[4])
    pitch_speed = float(msg.axes[1])
    yaw_speed = float(msg.axes[0])
    zero_state = int(msg.buttons[0])

rospy.init_node('joy_reader')

joy_sub = rospy.Subscriber('joy',Joy,callback)

pitch_pub = rospy.Publisher('signaturebot/arm_controller/position/pitch_joint/command', Float64, queue_size=1)
yaw_pub = rospy.Publisher('signaturebot/arm_controller/position/yaw_joint/command', Float64, queue_size=1)
ext_pub = rospy.Publisher('signaturebot/arm_controller/position/extension_joint/command', Float64, queue_size=1)

rate = rospy.Rate(50)

ext = 0.0
pitch = 0.0
yaw = 0.0

while not rospy.is_shutdown():
    if zero_state == 1:
        ext = 0.0
        pitch = 0.0
        yaw = 0.0
    ext = ext + ext_speed * 0.005
    pitch = pitch - pitch_speed * (3.14 / 32)
    yaw = yaw + yaw_speed * (3.14 / 32)

    pitch_pub.publish(pitch)
    yaw_pub.publish(yaw)
    ext_pub.publish(ext)

    rate.sleep()

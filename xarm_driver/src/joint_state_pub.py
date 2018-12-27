#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Jason PENG <jason@ufactory.cc>

# This is the joint state feedback process of real xArm7
import sys
import rospy
import copy
import argparse
from sensor_msgs.msg import JointState

# add python library path to system path
# sys.path.append("/Directory/to/the/xArm-Python-SDK/")
from xarm.wrapper import XArmAPI

PUB_TOPIC = 'joint_states'
PUB_RATE_HZ = 25
DOF = 7
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']\


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', dest='ip', default="192.168.1.11", help="your robot controler ip")
    parser.add_argument('__name:', help="ROS Node Name")
    parser.add_argument('__log:', help="ROS Node log file Name")

    args = parser.parse_args()

    # connect to the real xArm7 hardware with specified IP address 
    xarm = XArmAPI(args.ip, do_not_open=False)

    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('jnt_state_pub', anonymous=True)
    rate = rospy.Rate(PUB_RATE_HZ) # 10hz
    msg = JointState()
    msg.name = JOINT_NAMES
    now = rospy.get_rostime()
    prev_position = [0]*DOF
    msg.position = [0]*DOF
    msg.effort = [0]*DOF

    # TODO: check if /use_sim_time is set false
    while not rospy.is_shutdown():
        # TODO: get servo feedback and set the JointState msg
        prev_position = copy.deepcopy(msg.position)
        now = rospy.get_rostime()
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs

        msg.position = xarm.get_servo_angle()[1] # is_radian true by default

        msg.velocity = [ ((msg.position[i] - prev_position[i])*PUB_RATE_HZ) for i in range(DOF)]

        pub.publish(msg)
        rate.sleep()

    xarm.disconnect()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

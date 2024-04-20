#!/usr/bin/env python

from cartesian_interface.pyci_all import *
import rospy
import math
import numpy as np
from sympy import elliptic_e


def main():
    # obtain ci ros client
    ci = pyci.CartesianInterfaceRos()

    # define time between waypoints
    time = 1.0

    # initial left ee pose
    ee_left_pose_initial, _, _ = ci.getPoseReference('foo_ee_left')
    print(f"ee_left_pose_initial: {ee_left_pose_initial}")

    ee_left_pose_ref = ee_left_pose_initial.copy()
    ee_left_pose_ref.translation[2] -= 0.4

    ci.setTargetPose('foo_ee_left', ee_left_pose_ref, time)
    ci.waitReachCompleted('foo_ee_left')

    ee_left_pose_ref.translation[1] += 0.15

    ci.setTargetPose('foo_ee_left', ee_left_pose_ref, time)
    ci.waitReachCompleted('foo_ee_left')

    ee_left_pose_ref.translation[2] += 0.4 + 0.2

    ci.setTargetPose('foo_ee_left', ee_left_pose_ref, time)
    ci.waitReachCompleted('foo_ee_left')

    ci.setTargetPose('foo_ee_left', ee_left_pose_initial, time)
    ci.waitReachCompleted('foo_ee_left')

if __name__ == '__main__':
    rospy.init_node('cartesio_simple_poses')
    main()

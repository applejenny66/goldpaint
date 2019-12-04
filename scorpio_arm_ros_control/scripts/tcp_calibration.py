#!/usr/bin/env python

import sys, rospy, moveit_commander, random
import numpy as np
import copy
from geometry_msgs.msg import Pose, Quaternion, Point
from tf.transformations import quaternion_from_euler
from math import sin, cos

'''
TCP_CENTER_X = 0.96
TCP_CENTER_Y = 0.097
TCP_CENTER_Z = 0.115
'''
VEL_FAC = 0.50
ANGLE_BIAS = 0.43
BOUND_BIAS = 0.00

rospy.init_node('tcp_calibration')
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("arm")

# Get current pose
ps = Pose()
ps = group.get_current_pose().pose

position_arm = ()
position_arm = Point(ps.position.x, ps.position.y, ps.position.z)

waypoints = []

for theta in np.arange(0 + BOUND_BIAS, 6.28 - BOUND_BIAS, 0.05):
  orient_arm = Quaternion(*quaternion_from_euler(1.57 + (-cos(theta) * ANGLE_BIAS) , sin(theta) * ANGLE_BIAS, 0.00, 'syxz'))
  wpose = Pose()
  wpose.position = position_arm 
  wpose.orientation = orient_arm 
  waypoints.append(copy.deepcopy(wpose))

for theta in np.arange(6.28 - BOUND_BIAS, 0 + BOUND_BIAS, -0.05):
  orient_arm = Quaternion(*quaternion_from_euler(1.57 + (-cos(theta) * ANGLE_BIAS) , sin(theta) * ANGLE_BIAS, 0.00, 'syxz'))
  wpose = Pose()
  wpose.position = position_arm 
  wpose.orientation = orient_arm 
  waypoints.append(copy.deepcopy(wpose))


(plan1, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0);
plan1 = group.retime_trajectory(robot.get_current_state(), plan1, VEL_FAC)
print("fraction = {}".format(fraction))
group.execute(plan1)


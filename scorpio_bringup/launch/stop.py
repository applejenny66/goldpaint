#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import random
from math import pi
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionResult

group = None


def collidCB(data):
    global group
    print ('stop!')
    group.stop()
    rospy.sleep(2)

if __name__ == '__main__':
    rospy.init_node('stop_pose')
    moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    group = moveit_commander.MoveGroupCommander('arm')
    input = raw_input()

    collidsub = rospy.Subscriber("Collid", Bool, collidCB)

    rospy.spin()

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from math import pi
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionResult

group = None
Gstop = True

def collidCB(data):
    global Gstop
    Gstop = False

def go_idle(group):
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0.78
    joint_goal[2] = -1.54
    joint_goal[3] = 0
    joint_goal[4] = -0.13
    joint_goal[5] = 0
    plan_res = group.go(joint_goal)
    rospy.sleep(0.1)
    return plan_res

def go_left(group):
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -0.3
    joint_goal[2] = -0.7
    joint_goal[3] = 0
    joint_goal[4] = -1.0
    joint_goal[5] = 0
    plan_res = group.go(joint_goal)
    rospy.sleep(0.1)
    return plan_res

def go_right(group):
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = -1.0
    joint_goal[1] = -0.3
    joint_goal[2] = -0.7
    joint_goal[3] = 0
    joint_goal[4] = -1.0
    joint_goal[5] = 0
    plan_res = group.go(joint_goal)
    rospy.sleep(0.1)
    return plan_res

def planner(req):
    global group

    if req.data:
        result = go_zero(group)
    else:
        result = go_idel(group)
    rospy.sleep(1)
    res = SetBoolResponse()
    res.success = result
    return res

if __name__ == '__main__':
    rospy.init_node('repeat_pose')
    moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    group = moveit_commander.MoveGroupCommander('arm')
    group.set_goal_orientation_tolerance(0.035)
    group.set_goal_position_tolerance(0.03)

    #pose_srv = rospy.Service('repeat', SetBool, planner)


    voicepub = rospy.Publisher('/response', String, queue_size=10)
    rospy.loginfo('Ready to Plan.')
    go_left(group)
    rospy.sleep(3)
    rospy.loginfo('Press Enter to start')

    collidsub = rospy.Subscriber("Collid", Bool, collidCB)
    while not rospy.is_shutdown():
        if Gstop:
            go_left(group)
        else:
            voicepub.publish('please stay away')
            rospy.loginfo('Stopped.')
            rospy.loginfo('Press Enter to resume')
            Gstop = True
            input = raw_input()
        if Gstop:
            go_right(group)
        else:
            voicepub.publish('please stay away')
            rospy.loginfo('Stopped.')
            rospy.loginfo('Press Enter to resume')
            Gstop = True
            input = raw_input()
    go_idle(group)
    rospy.spin()

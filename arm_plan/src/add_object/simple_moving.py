#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from control_msgs.msg import GripperCommand
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import MoveGroupAction

group = None
gripperClient = None
gripperSrv = None
offset_z = 0.05
sim = True

def pickCB():
  global group, offset_z, sim

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x = -0.560
  pose_goal.position.y = 0.123
  pose_goal.position.z = 1.041
  pose_goal.orientation.x = -0.891
  pose_goal.orientation.y = -0.030
  pose_goal.orientation.z = 0.453
  pose_goal.orientation.w = 0.023
  group.set_pose_target(pose_goal)
  rospy.loginfo('GO')
  group.go()
  rospy.loginfo('Finish')

  return

if __name__ == '__main__':
  rospy.init_node('pick_and_place')

  #sim = rospy.get_param('~sim', True)
  #offset_z = rospy.get_param('~above_target_dist', 0.05)
  moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  #moveGroupClient.wait_for_server(rospy.Duration())
  #group = moveit_commander.MoveGroupCommander(
  # 'arm', '/scorpio/mmp0/robot_description', '/scorpio/mmp0')
  group = moveit_commander.MoveGroupCommander('arm')

  rospy.loginfo('Ready to plan.')
  raw_input()
  pickCB()
  rospy.spin()

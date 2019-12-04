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

def go_idel():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0
  joint_goal[2] = 0
  joint_goal[3] = 0
  joint_goal[4] = 0
  joint_goal[5] = 0
  plan_res = group.go(joint_goal)

  group.stop()
  return plan_res

def pickCB():
  global group, offset_z, sim

  ps = [Pose(), Pose(), Pose(), Pose()]
  pose_goal = Pose()
  InitPose = Pose()
  InitPose.position.x = -0.560; InitPose.position.y = 0.123; InitPose.position.z = 1.041
  InitPose.orientation.x = -0.891; InitPose.orientation.y = 0.0; InitPose.orientation.z = 0.453; InitPose.orientation.w = 0.0
  ps = [Pose()]
  ps[0].position.x = -0.381; ps[0].position.y = 0.680; ps[0].position.z = 0.847
  ps[0].orientation.x =  -0.715; ps[0].orientation.y = 0.699; ps[0].orientation.z = -0.005; ps[0].orientation.w = 0.033
  InitPose.position.x = -0.603; InitPose.position.y = 0.097; InitPose.position.z = 1.008
  InitPose.orientation.x = -0.891; InitPose.orientation.y = -0.030; InitPose.orientation.z = 0.453; InitPose.orientation.w = 0.023


  group.set_pose_target(InitPose)
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
  #  'arm', '/scorpio/mmp0/robot_description', '/scorpio/mmp0')
  group = moveit_commander.MoveGroupCommander('arm')
  group.set_goal_orientation_tolerance(0.1)
  group.set_goal_position_tolerance(0.01)

  rospy.loginfo('Ready to plan.')
  raw_input()
  #pickCB()
  go_idel()
  rospy.spin()

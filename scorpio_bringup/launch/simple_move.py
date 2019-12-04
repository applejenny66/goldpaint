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

def add_obj_A():
    scene = moveit_commander.PlanningSceneInterface()
    box_name = " "
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = 0.0
    box_pose.pose.position.y = 0.815
    box_pose.pose.position.z = 0.2
    box_pose.pose.orientation.w = 1.0
    box_name = "areaA_box"
    scene.add_box(box_name, box_pose, size=(1.5, 0.47, 0.87))

    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x=-0.8
    box_pose.pose.position.y=0.0
    box_pose.pose.position.z=0.25
    box_pose.pose.orientation.w = 1.0
    box_name = "wall"
    scene.add_box(box_name, box_pose, size=(0.3, 2.0, 1.0))

    return box_name

def go_idel():
  global group
  joint_goal = group.get_current_joint_values()
  joint_goal[0] = 0
  joint_goal[1] = 0.78
  joint_goal[2] = -1.54
  joint_goal[3] = 0
  joint_goal[4] = -0.13
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
  ps = [Pose(), Pose(), Pose(), Pose()]
  ps[0].position.x = -0.727; ps[0].position.y = 0.299; ps[0].position.z = 0.802
  ps[0].orientation.x = -0.999; ps[0].orientation.y = -0.042; ps[0].orientation.z = 0.004; ps[0].orientation.w = 0.013
  ps[0].position.x = -0.119; ps[0].position.y = 0.650; ps[0].position.z = 0.947
  ps[0].orientation.x = -0.700; ps[0].orientation.y = 0.714; ps[0].orientation.z = -0.011; ps[0].orientation.w = 0.027
  ps[1].position.x = 0.161; ps[1].position.y = -0.046; ps[1].position.z = 0.501
  ps[1].orientation.x = 0.320; ps[1].orientation.y = 0.947; ps[1].orientation.z = -0.041; ps[1].orientation.w = 0.004
  ps[1].position.x = 0.089; ps[1].position.y = -0.037; ps[1].position.z = 0.305
  ps[1].orientation.x = -0.374; ps[1].orientation.y = -0.928; ps[1].orientation.z = -0.003; ps[1].orientation.w = 0.007
  ps[2].position.x = 0.016; ps[2].position.y = -0.175; ps[2].position.z = 0.295
  ps[2].orientation.x = -0.374; ps[2].orientation.y = -0.928; ps[2].orientation.z = -0.003; ps[2].orientation.w = 0.007
  ps[2].position.x = 0.021; ps[2].position.y = -0.110; ps[2].position.z = 0.269
  ps[2].orientation.x = 0.042; ps[2].orientation.y = -0.999; ps[2].orientation.z = -0.011; ps[2].orientation.w = 0.007
  ps[0].position.x = -0.727; ps[0].position.y = 0.299; ps[0].position.z = 0.802
  ps[0].orientation.x = -0.999; ps[0].orientation.y = -0.042; ps[0].orientation.z = 0.004; ps[0].orientation.w = 0.013
  ps[1].position.x = 0.090; ps[1].position.y = -0.112; ps[1].position.z = 0.235
  ps[1].orientation.x = 0.042; ps[1].orientation.y = -0.999; ps[1].orientation.z = -0.011; ps[1].orientation.w = 0.007
  ps[0].position.x = -0.119; ps[0].position.y = 0.650; ps[0].position.z = 0.947
  ps[0].orientation.x = -0.700; ps[0].orientation.y = 0.714; ps[0].orientation.z = -0.011; ps[0].orientation.w = 0.027
  ps[0].position.x = -0.181; ps[0].position.y = 0.680; ps[0].position.z = 0.847
  ps[0].orientation.x =  -0.715; ps[0].orientation.y = 0.699; ps[0].orientation.z = -0.005; ps[0].orientation.w = 0.033
  ps[0].position.x = -0.838; ps[0].position.y = 0.280; ps[0].position.z = 0.505
  ps[0].orientation.x = -0.999; ps[0].orientation.y = -0.042; ps[0].orientation.z = 0.004; ps[0].orientation.w = 0.013
  ps[0].position.x = 0.061; ps[0].position.y = -0.110; ps[0].position.z = 0.269
  ps[0].orientation.x = 0.042; ps[0].orientation.y = -0.999; ps[0].orientation.z = -0.011; ps[0].orientation.w = 0.007
  ps[1].position.x = 0.086; ps[1].position.y = -0.105; ps[1].position.z = 0.340
  ps[1].orientation.x = -0.374; ps[1].orientation.y = -0.928; ps[1].orientation.z = -0.003; ps[1].orientation.w = 0.007
  ps[1].position.x = -0.857; ps[1].position.y = 0.260; ps[1].position.z = 0.540
  ps[1].orientation.x = -0.374; ps[1].orientation.y = -0.928; ps[1].orientation.z = -0.003; ps[1].orientation.w = 0.007
  #ps[1].position.x = 0.080; ps[1].position.y = 0.424; ps[1].position.z = 1.108
  #ps[1].orientation.x = -0.707; ps[1].orientation.y = 0.706; ps[1].orientation.z = 0.024; ps[1].orientation.w = 0.023
  #ps[1].position.x = 0.544; ps[1].position.y = -0.298; ps[1].position.z = 0.663
  #ps[1].orientation.x = -0.372; ps[1].orientation.y = -0.928; ps[1].orientation.z = -0.006; ps[1].orientation.w = 0.009
  ps[1].position.x = 0.095; ps[1].position.y = -0.656; ps[1].position.z = 1.053
  ps[1].orientation.x = 0.707; ps[1].orientation.y = 0.707; ps[1].orientation.z = -0.027; ps[1].orientation.w = 0.027
  ps[1].position.x = -0.252; ps[1].position.y = 0.053; ps[1].position.z = 0.530
  ps[1].orientation.x = 0.905; ps[1].orientation.y = -0.425; ps[1].orientation.z = -0.035; ps[1].orientation.w = 0.003
  ps[1].position.x = -0.288; ps[1].position.y = -0.239; ps[1].position.z = 0.655
  ps[1].orientation.x = 1.000; ps[1].orientation.y = 0.002; ps[1].orientation.z = -0.015; ps[1].orientation.w = 0.004
  ps[0].position.x = 0.095; ps[0].position.y = -0.706; ps[0].position.z = 0.753
  ps[0].orientation.x = 0.707; ps[0].orientation.y = 0.707; ps[0].orientation.z = -0.027; ps[0].orientation.w = 0.027
  #ps[1].position.x = 0.245; ps[1].position.y = -0.706; ps[1].position.z = 0.853
  #ps[1].orientation.x = 0.707; ps[1].orientation.y = 0.707; ps[1].orientation.z = -0.027; ps[1].orientation.w = 0.027
  ps[0].position.x = 0.178; ps[0].position.y = -0.491; ps[0].position.z = 0.688
  ps[0].orientation.x = 0.757; ps[0].orientation.y = 0.653; ps[0].orientation.z = -0.010; ps[0].orientation.w = 0.017
  #ps[4].position.x = -0.288; ps[4].position.y = -0.189; ps[4].position.z = 0.475
  #ps[4].orientation.x = 1.000; ps[4].orientation.y = 0.002; ps[4].orientation.z = -0.015; ps[4].orientation.w = 0.004
  group.set_pose_target(ps[0])
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
  #  'arm', '/robot_description', '')
  group = moveit_commander.MoveGroupCommander('arm')
  group.set_goal_orientation_tolerance(0.01)
  group.set_goal_position_tolerance(0.01)

  rospy.loginfo('Ready to plan.')
  #add_obj_A()
  raw_input()
  pickCB()
  #go_idel()
  rospy.spin()

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from arm_plan.srv import PoseSrv, PoseSrvResponse
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory

def add_obj(self):
     box_name = ' '
     scene = moveit_commander.PlanningSceneInterface()

     box_pose = geometry_msgs.msg.PoseStamped()
     box_pose.header.frame_id = "base_link"
     box_pose.pose.position.x=0.0
     box_pose.pose.position.y=-0.843 #0.65
     box_pose.pose.position.z=0.25
     box_pose.pose.orientation.w = 1.0
     box_name = "areaB_box"
     scene.add_box(box_name, box_pose, size=(1.152, 0.326, 0.898))

     return

def go_to_pose_goal(x ,y ,z):
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = x #0.0499
    pose_goal.position.y = y #0.6 
    pose_goal.position.z = z #1.1
    pose_goal.orientation = Quaternion(*quaternion_from_euler(1.75, 0.00, 0.00, 'szyx'))
    group.set_pose_target(pose_goal)	

    plan = group.go(wait=True)


def planner(req):
  	print('Receive pose')
	print('pick_pose = ({}, {}, {})'.format(
    req.pose.position.x, req.pose.position.y, req.pose.position.z))
	x = req.pose.position.x 
  	y = req.pose.position.y
  	z = req.pose.position.z
	add_obj(' ')

  	go_to_pose_goal(x ,y ,z)
  	rospy.sleep(5)
  	print('============ Above Scenario Completed ==========\n')
  	return PoseSrvResponse(True) 

if __name__ == '__main__':
  rospy.init_node('attacking_shipping_pose')

  pose_srv = rospy.Service('attacking_shipping_pose', PoseSrv, planner)

  print('Go to the pose.')
  rospy.spin()

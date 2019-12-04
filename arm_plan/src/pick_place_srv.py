#!/usr/bin/env python

import rospy
import sys
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
from arm_plan.srv import PoseSrv, PoseSrvResponse
from robotiq_2f_gripper_control.srv import gripper, gripperResponse, gripperRequest

#from tm_msgs.srv import SetIO
#from neuronbot_msgs.srv import PickPlace, PickPlaceResponse

group = None
add_odj = None
offset_z = 0.1 #10cm

class add_object(object):
    def __init__(self):
        super(add_object, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()

    def add_obj_A(self):
        box_name = " "

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.815
        box_pose.pose.position.z = 0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.91, 0.47, 0.87))

        return box_name

    def add_obj_B(self):
        box_name = " "

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = -0.843
        box_pose.pose.position.z = 0.25
        box_pose.pose.orientation.w = 1.0
        box_name = "areaB_box"
        self.scene.add_box(box_name, box_pose, size=(1.152, 0.326, 0.898))

        return box_name

    def add_obj_C(self):
        box_name = " "

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = -0.93
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "areaC_box"
        self.scene.add_box(box_name, box_pose, size=(0.6, 1.0, 0.729))

        return box_name

    def remove_box(self, re_box_name):
        if re_box_name == "box":
            self.scene.remove_world_object(re_box_name)
        elif re_box_name == "areaB_box":
            self.scene.remove_world_object(re_box_name)
        elif re_box_name == "areaC_box":
            self.scene.remove_world_object(re_box_name)

        return

def pick_obj(req):
  global group, offset_z, add_odj
  print(' pick_pose = ({}, {}, {})'.format(
    req.pose.position.x, req.pose.position.y, req.pose.position.z))

  # add box
  rospy.loginfo('add object')
  if req.str_box_ind == 'a':
      re_box_name = add_odj.add_obj_A()
  elif req.str_box_ind == 'b':
      re_box_name = add_odj.add_obj_B()
  elif req.str_box_ind == 'c':
      re_box_name = add_odj.add_obj_C()

  # ViaPoints
  above_pick_target_ps = Pose()

  # go to above_pick_target_ps
  above_pick_target_ps.position.x = req.pose.position.x
  above_pick_target_ps.position.y = req.pose.position.y
  above_pick_target_ps.position.z = req.pose.position.z + offset_z
  above_pick_target_ps.orientation = Quaternion(*quaternion_from_euler(1.57, 3.14, 0.00, 'szyx'))
  group.set_pose_target(above_pick_target_ps)
  group.go()

  # pick target
  group.set_pose_target(req.pose)
  group.go()

  # add gripper funtion
  rospy.loginfo('close gripper')
  action('c')
  rospy.sleep(0.1)

  # remove_box
  rospy.loginfo('remove_box')
  add_odj.remove_box(re_box_name)

  return PoseSrvResponse(True)

def place_obj(req):
  global group, offset_z, add_odj
  print(' pick_pose = ({}, {}, {})'.format(
    req.pose.position.x, req.pose.position.y, req.pose.position.z))

  # add box
  rospy.loginfo('add object')
  if req.str_box_ind == 'a':
      re_box_name = add_odj.add_obj_A()
  elif req.str_box_ind == 'b':
      re_box_name = add_odj.add_obj_B()
  elif req.str_box_ind == 'c':
      re_box_name = add_odj.add_obj_C()

  # ViaPoints
  above_place_target_ps = Pose()

  # go to above_pick_target_ps
  above_place_target_ps.position.x = req.pose.position.x
  above_place_target_ps.position.y = req.pose.position.y
  above_place_target_ps.position.z = req.pose.position.z + offset_z
  above_place_target_ps.orientation = Quaternion(*quaternion_from_euler(1.57, 3.14, 0.00, 'szyx'))
  group.set_pose_target(above_place_target_ps)
  group.go()

  # place target
  group.set_pose_target(req.pose)
  group.go()

  # add gripper funtion
  rospy.loginfo('open gripper')
  action('o')
  rospy.sleep(0.1)

  # remove_box
  rospy.loginfo('remove_box')
  add_odj.remove_box(re_box_name)

  return PoseSrvResponse(True)

if __name__ == '__main__':
  rospy.init_node('pick_place')

  #sim = rospy.get_param('~sim', True)
  #offset_z = rospy.get_param('~above_target_dist', 0.05)
  #moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  #moveGroupClient.wait_for_server(rospy.Duration())
  #group = moveit_commander.MoveGroupCommander(
  # 'arm', '/%s/robot_description'%rospy.get_namespace(), '/neuronbot/mmp0')
  group = moveit_commander.MoveGroupCommander('arm')
  add_odj = add_object()

  rospy.logwarn("Waiting for gripper!")
  rospy.wait_for_service('grippercontroller_srv')

  action = rospy.ServiceProxy('grippercontroller_srv', gripper)


  rospy.loginfo('Ready to plan.')
  pick_srv = rospy.Service('pick', PoseSrv, pick_obj)
  place_srv = rospy.Service('place', PoseSrv, place_obj)

  rospy.spin()

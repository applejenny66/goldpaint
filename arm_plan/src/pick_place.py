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
from arm_plan.srv import PickPlace, PickPlaceResponse, PickPlaceRequest
from robotiq_2f_gripper_control.srv import gripper, gripperResponse, gripperRequest


group = None
add_odj = None
req = None
plan_res = None
offset_z = 0.05 #10cm

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
        box_name = "areaA_box"
        self.scene.add_box(box_name, box_pose, size=(0.91, 0.47, 0.87))

	box_pose.header.frame_id = "base_link"
	box_pose.pose.position.x=-0.8
	box_pose.pose.position.y=0.0
	box_pose.pose.position.z=0.25
	box_pose.pose.orientation.w = 1.0
	box_name = "wall"
	self.scene.add_box(box_name, box_pose, size=(0.3, 1.0, 0.95))

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

        box_pose.header.frame_id = "base_link"
	box_pose.pose.position.x=-1.0
	box_pose.pose.position.y=-0.32
	box_pose.pose.position.z=0.75
	box_pose.pose.orientation.w = 1.0
	box_name = "computer"
	self.scene.add_box(box_name, box_pose, size=(0.42, 0.35, 0.42))

        return box_name

    def remove_box(self, re_box_name):
        if re_box_name == "areaA_box":
            self.scene.remove_world_object(re_box_name)
        elif re_box_name == "areaB_box":
            self.scene.remove_world_object(re_box_name)
        elif re_box_name == "areaC_box":
            self.scene.remove_world_object(re_box_name)

        return

    def pick_obj(self, pick_ps):
        global offset_z
        print(' pick_pose = ({}, {}, {})'.format(
          pick_ps.position.x, pick_ps.position.y, pick_ps.position.z))

        # ViaPoints
        above_pick_target_ps = Pose()

        # go to above_pick_target_ps
        above_pick_target_ps = pick_ps
        above_pick_target_ps.position.z = pick_ps.position.z + offset_z
        rospy.loginfo("above_pick_target_ps")
        group.set_pose_target(above_pick_target_ps)
        group.go()
        plan_res = group.go()
	if not plan_res == True:
		return plan_res
	rospy.sleep(1)

        # pick target
	group.set_pose_target(pick_ps)
        plan_res = group.go()

	# add gripper funtion
	if plan_res == True:
        	rospy.loginfo('close gripper')
        	action('c')
        	rospy.sleep(0.2)
		#return plan_res
	else:
        	return plan_res

	# go back to above_pick_target_ps
        above_place_target_ps = pick_ps
        above_place_target_ps.position.z = pick_ps.position.z + offset_z
        group.set_pose_target(above_place_target_ps)
        rospy.loginfo("above_pick_target_ps")
        group.go()

	return True


    def place_obj(self, place_ps):
        global offset_z
        print(' place_pose = ({}, {}, {})'.format(
          place_ps.position.x, place_ps.position.y, place_ps.position.z))

        # ViaPoints
        above_place_target_ps = Pose()

        # go to above_pick_target_ps
        above_place_target_ps = place_ps
        above_place_target_ps.position.z = place_ps.position.z + offset_z
        group.set_pose_target(above_place_target_ps)
        plan_res = group.go()
        if not plan_res == True:
            return plan_res
	rospy.sleep(1)

        # place target
        group.set_pose_target(place_ps)
        plan_res = group.go()

        # add gripper funtion
	if plan_res == True:
		rospy.loginfo('open gripper')
        	action('o')
        	rospy.sleep(0.1)
		#return plan_res
	else:
        	return plan_res

	# go back to above_pick_target_ps
        above_place_target_ps = place_ps
        above_place_target_ps.position.z = place_ps.position.z + offset_z
        #print (place_ps)
        #print (above_place_target_ps)
        group.set_pose_target(above_place_target_ps)
        group.go()

	return True

def planner(req):
    global add_odj, group

    # add box
    rospy.loginfo('add object')
    if req.str_box_ind == 'a':
        re_box_name = add_odj.add_obj_A()
    elif req.str_box_ind == 'b':
        re_box_name = add_odj.add_obj_B()
    elif req.str_box_ind == 'c':
        re_box_name = add_odj.add_obj_C()

    # go to pick pose
    res_pick = add_odj.pick_obj(req.pick_pose)
    rospy.sleep(0.5)
    if res_pick == True:
    	rospy.loginfo('complete picking')
    else:
	return PickPlaceResponse(False)

    rospy.sleep(5)
    # go to place pose
    res_place = add_odj.place_obj(req.place_pose)
    rospy.sleep(0.5)
    if res_place == True:
	rospy.loginfo('complete placeing')
    else:
	return PickPlaceResponse(False)

    # remove_box
    rospy.loginfo('remove_box')
    add_odj.remove_box(re_box_name)

    return PickPlaceResponse(True)


if __name__ == '__main__':
  rospy.init_node('pick_place')

  #moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  #moveGroupClient.wait_for_server(rospy.Duration())
  #group = moveit_commander.MoveGroupCommander(
  # 'arm', '/%s/robot_description'%rospy.get_namespace(), '/neuronbot/mmp0')
  moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
  #moveGroupClient.wait_for_server(rospy.Duration())
  group = moveit_commander.MoveGroupCommander(
    'arm', '/scorpio/mmp0/robot_description', '/scorpio/mmp0')
  group.set_goal_orientation_tolerance(0.1)
  group.set_goal_position_tolerance(0.01)
  add_odj = add_object()

  rospy.logwarn("Waiting for gripper!")
  rospy.wait_for_service('grippercontroller_srv')
  action = rospy.ServiceProxy('grippercontroller_srv', gripper)

  # strat pick and place pipline
  rospy.loginfo('Ready to plan.')
  pick_srv = rospy.Service('pick_and_place', PickPlace, planner)

  rospy.spin()

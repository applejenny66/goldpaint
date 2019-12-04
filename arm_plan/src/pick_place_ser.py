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
offset_z = 0.07 #10cm

class add_object(object):
    def __init__(self):
        super(add_object, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()

    def add_obj_A(self):
        """
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.815
        box_pose.pose.position.z = 0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "areaA_box"
        self.scene.add_box(box_name, box_pose, size=(1.5, 0.47, 0.87))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = -0.8
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.25
        box_pose.pose.orientation.w = 1.0
        box_name = "wall"
        self.scene.add_box(box_name, box_pose, size=(0.3, 2.0, 1.7))
        """
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.23+0.09
        box_pose.pose.position.y=-0.68
        box_pose.pose.position.z=0.648
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        self.scene.add_box(box_name, box_pose, size=(0.05, 0.47, 0.04))


        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=-0.23+0.09
        box_pose.pose.position.y=-0.68
        box_pose.pose.position.z=0.648
        box_pose.pose.orientation.w = 1.0
        box_name = "box1"
        self.scene.add_box(box_name, box_pose, size=(0.05, 0.47, 0.04))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=-0.68
        box_pose.pose.position.z=0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "base"
        self.scene.add_box(box_name, box_pose, size=(1.5, 0.47, 0.85))

        return "add_obj_A"

    def add_obj_B(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.75
        box_pose.pose.position.z = 0.25
        box_pose.pose.orientation.w = 1.0
        box_name = "areaB_box"
        self.scene.add_box(box_name, box_pose, size=(1.152, 0.326, 0.898))

        return "add_obj_B"

    def add_obj_C(self):
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.570
        box_pose.pose.position.z = 0.5
        box_pose.pose.orientation.w = 1.0
        box_name = "302door"
        self.scene.add_box(box_name, box_pose, size=(2.0, 0.088, 1.5))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=-0.4
        box_pose.pose.position.y=-0.4
        box_pose.pose.position.z=1.25
        box_pose.pose.orientation.w = 1.0
        box_name = "302wall"
        self.scene.add_box(box_name, box_pose, size=(0.7, 0.142, 3.0))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=-0.77
        box_pose.pose.position.z=0.1235
        box_pose.pose.orientation.w = 1.0
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=(1.8, 0.542, 0.739))

        return "add_obj_C"

    def remove_box(self, re_box_name):
        if re_box_name == "add_obj_A":
            self.scene.remove_world_object("box")
            self.scene.remove_world_object("box1")
            self.scene.remove_world_object("base")
            #self.scene.remove_world_object("wall")
        elif re_box_name == "add_obj_B":
            self.scene.remove_world_object("areaB_box")
        elif re_box_name == "add_obj_C":
            self.scene.remove_world_object("302door")
            self.scene.remove_world_object("302wall")
            self.scene.remove_world_object("table")

    def go_idel(self):
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0.78
        joint_goal[2] = -1.54
        joint_goal[3] = 0
        joint_goal[4] = -0.13
        joint_goal[5] = 0
        plan_res = group.go(joint_goal)
        return plan_res

    def above_ps(self, pose_point):
        global offset_z

        # ViaPiont
        above_pose_point = Pose()

        #above_pose_point = pose_point
        above_pose_point.position.x = pose_point.position.x
        above_pose_point.position.y = pose_point.position.y
        above_pose_point.position.z = pose_point.position.z + offset_z
        above_pose_point.orientation = pose_point.orientation
        print(pose_point)
        print(above_pose_point)
        rospy.loginfo("go above_target_pose")
        group.set_pose_target(above_pose_point)
        plan_res = group.go()
        group.stop()
        rospy.loginfo("complete go above_target_pose")
        return plan_res

    def pick_obj(self, pick_ps):
        print(' pick_pose = ({}, {}, {})'.format(
          pick_ps.position.x, pick_ps.position.y, pick_ps.position.z))

        # pick target
        group.set_pose_target(pick_ps)
        plan_res = group.go()
        group.stop()
        #rospy.sleep(0.5)

        # add gripper funtion
        if plan_res == True:
            rospy.loginfo('close gripper')
            action('c')
            rospy.sleep(2)
            return plan_res
        else:
            return plan_res

    def place_obj(self, place_ps):
        print(' place_pose = ({}, {}, {})'.format(
          place_ps.position.x, place_ps.position.y, place_ps.position.z))

        # place target
        group.set_pose_target(place_ps)
        plan_res = group.go()
        group.stop()
        #rospy.sleep(0.5)

        # add gripper funtion
        if plan_res == True:
            rospy.loginfo('open gripper')
            action('o')
            rospy.sleep(2)
            return plan_res
        else:
            return plan_res

def planner(req):
    global add_odj, group
    res = PickPlaceResponse()
    # add box
    rospy.loginfo('add object')
    if req.str_box_ind[0] == 'a':
        re_box_name = add_odj.add_obj_A()
    elif req.str_box_ind[0] == 'c':
        re_box_name = add_odj.add_obj_B()
    elif req.str_box_ind[0] == 'b':
        re_box_name = add_odj.add_obj_C()

    # go to pick pose
    if req.str_box_ind[1] != '2':
        add_odj.above_ps(req.pick_pose)
        #rospy.sleep(1)

        res_pick = add_odj.pick_obj(req.pick_pose)
        if res_pick == True:
            rospy.loginfo('complete picking')
        else:
            res.result = False
            res.state = 'pick'
            return res

        add_odj.above_ps(req.pick_pose)
        #rospy.sleep(1)

    # go ideal pose
    #add_odj.go_idel()
    #rospy.sleep(1)

    # go to place pose
    add_odj.above_ps(req.place_pose)
    #rospy.sleep(12)
    res_place = add_odj.place_obj(req.place_pose)
    if res_place == True:
        rospy.loginfo('complete placeing')
    else:
        res.result = False
        res.state = 'place'
        return res
    add_odj.above_ps(req.place_pose)
    #rospy.sleep(1)

	# remove_box
    rospy.loginfo('remove_box')
    add_odj.remove_box(re_box_name)

    res.result = True
    res.state = 'ya'
    return res


if __name__ == '__main__':
    rospy.init_node('pick_place')
    #moveGroupClient = actionlib.SimpleActionClient('move_group', MoveGroupAction)
    #moveGroupClient.wait_for_server(rospy.Duration())
    #group = moveit_commander.MoveGroupCommander(
    # 'arm', '/%s/robot_description'%rospy.get_namespace(), '/neuronbot/mmp0')
    group = moveit_commander.MoveGroupCommander('arm')
    group.set_goal_orientation_tolerance(0.01)
    group.set_goal_position_tolerance(0.01)

    rospy.wait_for_service('grippercontroller_srv')
    action = rospy.ServiceProxy('grippercontroller_srv', gripper)
    add_odj = add_object()

    # strat pick and place pipline
    rospy.loginfo('Ready to plan.')
    pick_srv = rospy.Service('pick_and_place', PickPlace, planner)

    rospy.spin()

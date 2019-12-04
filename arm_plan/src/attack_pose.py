#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from arm_plan.srv import PoseSrv, PoseSrvResponse
from std_msgs.msg import String, Bool
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import MoveGroupAction
from robotiq_2f_gripper_control.srv import gripper, gripperResponse, gripperRequest


add_odj = None
Ggroup = None
plan_res =None

def emergeStop(data):
    global Ggroup
    Ggroup.stop()

class add_object_atc(object):
    def __init__(self):
        super(add_object_atc, self).__init__()

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
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=-0.68
        box_pose.pose.position.z=0.2
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
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
        box_pose.pose.position.y=-0.57
        box_pose.pose.position.z=1.25
        box_pose.pose.orientation.w = 1.0
        box_name = "302wall"
        self.scene.add_box(box_name, box_pose, size=(0.7, 0.142, 3.0))

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose.position.x=0.0
        box_pose.pose.position.y=-0.77
        box_pose.pose.position.z=0.15
        box_pose.pose.orientation.w = 1.0
        box_name = "table"
        self.scene.add_box(box_name, box_pose, size=(1.8, 0.542, 0.75))

        return "add_obj_C"


    def go_to_pose_goal(self, pose, group):

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = pose
        print(pose_goal)
        group.set_pose_target(pose_goal)
        plan_res = group.go()
        rospy.sleep(1)

        return plan_res

    def go_to_idle(self, group):
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

def planner(req):
    global add_odj, Ggroup

    # add box
    rospy.loginfo('add object')
    if req.str_box_ind == 'i':
        res = add_odj.go_to_idle(Ggroup)
        return res
    elif req.str_box_ind == 'a':
        re_box_name = add_odj.add_obj_A()
    elif req.str_box_ind == 'c':
        re_box_name = add_odj.add_obj_B()
    elif req.str_box_ind == 'b':
        re_box_name = add_odj.add_obj_C()

    # go to attack pose
    print('pick_pose = ({}, {}, {})'.format(
        req.pose.position.x, req.pose.position.y, req.pose.position.z))

    res = add_odj.go_to_pose_goal(req.pose, Ggroup)

    print(res)
    if res == True:
        return PoseSrvResponse(True)
    else:
	      return PoseSrvResponse(False)

if __name__ == '__main__':
    rospy.init_node('attacking_pose')

    add_odj = add_object_atc()
    #Ggroup = moveit_commander.MoveGroupCommander('arm', '/scorpio/mmp0/robot_description', '/scorpio/mmp0')
    Ggroup = moveit_commander.MoveGroupCommander('arm')
    Ggroup.set_goal_orientation_tolerance(0.02)
    Ggroup.set_goal_position_tolerance(0.02)

    rospy.wait_for_service('grippercontroller_srv')
    action = rospy.ServiceProxy('grippercontroller_srv', gripper)

    #emerge_stop_sub = rospy.Subscriber("/Torque", Bool, emergeStop)
    pose_srv = rospy.Service('attacking_pose', PoseSrv, planner)

    rospy.spin()

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
from trajectory_msgs.msg import JointTrajectory

class add_object(object):
	def __init__(self):
		super(add_object, self).__init__()
		
		moveit_commander.roscpp_initialize(sys.argv)
    		rospy.init_node('add_object', anonymous=True)

		robot = moveit_commander.RobotCommander()
		scene = moveit_commander.PlanningSceneInterface()

		group_name = "arm"
    		group = moveit_commander.MoveGroupCommander(group_name)

		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


		planning_frame = group.get_planning_frame()
    		#print ("=== Reference frame: %s" % planning_frame)

    		eef_link = group.get_end_effector_link()
    		#print ("=== End effector: %s" % eef_link)

    		group_names = robot.get_group_names()
    		#print ("=== Robot Groups:", robot.get_group_names())

    		#print ("=== Printing robot state")
    		#print robot.get_current_state()

		print ("===Printing Robot End-Effector position")
		print group.get_current_pose()
		print ("===Printing Robot End-Effector RPY")
		print group.get_current_rpy()


		#variables
		self.box_name = ''
		self.robot = robot
		self.scene = scene
		self.group = group
		self.display_trajectory_publisher = display_trajectory_publisher
    		self.planning_frame = planning_frame
    		self.eef_link = eef_link
    		self.group_names = group_names

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
		box_name = self.box_name
    		scene = self.scene

		start = rospy.get_time()
    		seconds = rospy.get_time()
    		while (seconds - start < timeout) and not rospy.is_shutdown():
      			# Test if the box is in attached objects
      			attached_objects = scene.get_attached_objects([box_name])
      			is_attached = len(attached_objects.keys()) > 0

      			# Test if the box is in the scene.
      			# Note that attaching the box will remove it from known_objects
      			is_known = box_name in scene.get_known_object_names()

      			# Test if we are in the expected state
      			if (box_is_attached == is_attached) and (box_is_known == is_known):
        			return True

      			# Sleep so that we give other threads time on the processor
      			rospy.sleep(0.1)
      			seconds = rospy.get_time()

    		# If we exited the while loop without returning then we timed out
    		return False

	def add_obj(self):
		box_name = self.box_name
		scene = self.scene

		box_pose = geometry_msgs.msg.PoseStamped()
		box_pose.header.frame_id = "base_link"
		box_pose.pose.position.x=0.0
		box_pose.pose.position.y=-0.6
		box_pose.pose.position.z=0.25
		box_pose.pose.orientation.w = 1.0
    		box_name = "box"
    		scene.add_box(box_name, box_pose, size=(1.152, 0.326, 0.898))

		self.box_name = box_name
		self.scene = scene
		return
    		#return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	def go_to_pose_goal(self):
		group = self.group

		pose_goal = geometry_msgs.msg.Pose()
    		pose_goal.position.x = 0.326
    		pose_goal.position.y = -0.496
    		pose_goal.position.z = 1.14
		pose_goal.orientation = Quaternion(*quaternion_from_euler(1.75, 3.14, 0.00, 'szyx'))
    		group.set_pose_target(pose_goal)

		plan = group.go(wait=True)
		group.stop()
		group.clear_pose_targets()	

def main():
	try:
    		tutorial = add_object()

    		print "============ Press `Enter` to add box and object to the planning scene ..."
    		raw_input()
		tutorial.add_obj()
		#print "============ Press `Enter` to execute a movement using a pose goal ..."
    		#raw_input()
    		#tutorial.go_to_pose_goal()

	except rospy.ROSInterruptException:
    		return
  	except KeyboardInterrupt:
    		return

if __name__ == '__main__':
  main()

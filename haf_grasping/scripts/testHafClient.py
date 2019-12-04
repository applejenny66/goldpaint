#!/usr/bin/env python

import actionlib
import rospy
from sensor_msgs.msg import PointCloud2
from haf_grasping.msg import CalcGraspPointsServerAction, CalcGraspPointsServerGoal
from geometry_msgs.msg import Point, Vector3

pcl = None

def pclCB(msg):
  global pcl
  pcl = msg
<<<<<<< HEAD
=======
  print "get"
>>>>>>> 2b5fc492c37e3b2675c4d2dbec6a2ce5b143240d

if __name__ == '__main__':
  rospy.init_node('haf_test_client')
  rospy.Subscriber('/c1/camera/depth/points', PointCloud2, pclCB)

  ac = actionlib.SimpleActionClient('/calc_grasppoints_svm_action_server', CalcGraspPointsServerAction) 
<<<<<<< HEAD
=======
  print "Waiting for action server to start."
>>>>>>> 2b5fc492c37e3b2675c4d2dbec6a2ce5b143240d
  ac.wait_for_server()
  goal = CalcGraspPointsServerGoal()
  goal.graspinput.input_pc = pcl
  goal.graspinput.goal_frame_id = '/c1_tf/camera_link'

  pt = Point()
  pt.x = 1.5
  pt.y = -0.1
  pt.z = -0.2
  goal.graspinput.grasp_area_center = pt
  goal.graspinput.grasp_area_length_x = 32.0
  goal.graspinput.grasp_area_length_y = 44.0
  goal.graspinput.max_calculation_time = rospy.Duration.from_sec(3.0) 
  goal.graspinput.show_only_best_grasp = False

  vc = Vector3()
  vc.x = -1
  vc.y = 0
  vc.z = 0
  goal.graspinput.approach_vector = vc
  goal.graspinput.gripper_opening_width = 1

  ac.send_goal(goal)
  ac.wait_for_result()
  print ac.get_result()


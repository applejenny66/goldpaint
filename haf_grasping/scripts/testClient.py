#!/usr/bin/env python

import rospy
from haf_grasping.srv import GraspPoseEst_direct
from geometry_msgs.msg import Point

if __name__ == '__main__':
  rospy.init_node('haf_test_client')

  rospy.wait_for_service('/haf_grasping/GraspPoseEst')
  hafReq = rospy.ServiceProxy('/haf_grasping/GraspPoseEst', GraspPoseEst_direct)

  x_temp = 0.15
  y_temp = 0.15

  pt1 = Point()
  pt1.x = x_temp
  pt1.y = y_temp 

  pt2 = Point()
  pt2.x = x_temp + 0.05
  pt2.y = y_temp + 0.05

  resp = hafReq(pt1, pt2)

#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose
from arm_plan.srv import PickPlace, PickPlaceResponse

def Pose_client(pick_pose, place_pose, str_box_ind):
    rospy.wait_for_service('pick_and_place')
    #rospy.wait_for_service('place')
    try:
      gopose = rospy.ServiceProxy('pick_and_place', PickPlace)
      #gopose = rospy.ServiceProxy('place', PoseSrv)
      resp = gopose(pick_pose, place_pose, str_box_ind)
      return resp.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
   print("start")

   ps1 = Pose()
   ps1.position.x = -0.3#0.01 #0.05
   ps1.position.y = 0.5#0.2 #0.5
   ps1.position.z = 0.5#1.0   #1.0

   ps2 = Pose()
   ps2.position.x = 0.3#0.01 #0.05
   ps2.position.y = 0.5#0.2 #0.5
   ps2.position.z = 0.5#1.0   #1.0

   str_command = 'a'

   Pose_client(ps1, ps2, str_command)

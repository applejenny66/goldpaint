#!/usr/bin/env python

import sys
import rospy

from geometry_msgs.msg import Pose
from arm_plan.srv import PoseSrv, PoseSrvResponse

def Pose_client(pose, str_box_ind):
    rospy.wait_for_service('attacking_pose')

    try:
      gopose = rospy.ServiceProxy('attacking_pose', PoseSrv)
      resp = gopose(pose, str_box_ind)
      return resp.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
   print("start")

   ps1 = Pose()
   ps1.position.x = 0.01
   ps1.position.y = 0.2
   ps1.position.z = 1.0

   str_command = 'c'

   Pose_client(ps1, str_command)

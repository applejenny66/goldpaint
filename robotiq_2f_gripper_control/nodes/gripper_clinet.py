#!/usr/bin/env python

import sys
import rospy
from robotiq_2f_gripper_control.srv import gripper,  gripperResponse,gripperRequest


def gripper_client(ch):
    rospy.wait_for_service('grippercontroller_srv')

    try:
      action = rospy.ServiceProxy('grippercontroller_srv', gripper)

      resp = action(ch)
      return resp.success
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    print("start control gripper")
    print("input your command: ")
    ch = raw_input()

    gripper_client(ch)

#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg
from time import sleep
from robotiq_2f_gripper_control.srv import gripper, gripperResponse,gripperRequest

def reset():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 0
    command.rGTO = 0
    command.rSP  = 0
    command.rFR  = 0

    return command

def activate():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255  # the velocity of gripper, range 0~255
    command.rFR  = 150  # the force of the gripper, range 0~255

    return command

def closegripper():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rPR = 255
    command.rFR  = 150  # the force of the gripper, range 0~255

    return command

def opengripper():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rPR = 50
    command.rFR  = 150  # the force of the gripper, range 0~255

    return command


def action(req):

    print('req_msgs = '.format(req.str_com))
    ch = req.str_com
    if ch == 'c':
       com_colse = closegripper()
       pub.publish(com_colse)

    elif ch == 'o':
       com_open = opengripper()
       pub.publish(com_open)

    rospy.sleep(1)

    return gripperResponse(True)

if __name__ == '__main__':
    rospy.init_node('Robotiq2FGripperController')

    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size = 10)
    rospy.sleep(2)

    # init gripper
    com_res = reset()
    pub.publish(com_res)

    rospy.sleep(0.5)

    com_act = activate()
    pub.publish(com_act)

    rospy.sleep(0.5)

    rospy.Service('grippercontroller_srv', gripper, action)

    rospy.spin()

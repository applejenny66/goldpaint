#!/usr/bin/env python

import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
roslib.load_manifest('robotiq_modbus_rtu')
import rospy
import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import os, sys
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input  as inputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from time import sleep

def mainLoop(device):
    
    #Gripper is a 2F with a TCP connection
    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper()
    gripper.client = robotiq_modbus_rtu.comModbusRtu.communication()

    #We connect to the address received as an argument
    gripper.client.connectToDevice(device)

    rospy.init_node('robotiq2FGripper')

    #The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input)

    #The Gripper command is received from the topic named 'Robotiq2FGripperRobotOutput'
    rospy.Subscriber('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, gripper.refreshCommand)
    

    #We loop
    while not rospy.is_shutdown():

      #Get and publish the Gripper status
      status = gripper.getStatus()
      pub.publish(status)     

      #Wait a little
      #rospy.sleep(0.05)

      #Send the most recent command
      gripper.sendCommand()

      #Wait a little
      #rospy.sleep(0.05)

def gipper_init():
    command = outputMsg.Robotiq2FGripper_robot_output();
    
    while not rospy.is_shutdown():

        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

        rospy.sleep(0.1)
            
if __name__ == '__main__':
    try:
        mainLoop(sys.argv[1])

        rospy.sleep(1)

        gipper_init()

    except rospy.ROSInterruptException: pass

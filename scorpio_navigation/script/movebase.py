#!/usr/bin/env python

import actionlib
import rospy
import smach
import smach_ros
import os
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped ,Twist
from ira_factory_msgs.srv import TaskExe, TaskExeResponse, UpdateRobotStatus
from ira_factory_msgs.msg import RobotStatus
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathFeedback, ExePathResult
from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from smach_ros import SimpleActionState
from pmc_navigation.srv import navigoal, navigoalResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from ira_factory_msgs.srv import RobotTF


mokuteke = None
tfReq = None
#rotateVel = 0.10 #0.10 0.25
transVel = 0.15 #0.05  0.25
angleTH = 0.05 #0.05



def NavStateMachine():
    global mokuteke
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.z = 0
    if(mokuteke==1):
    	goal.target_pose.pose.position.x = -1.84
    	goal.target_pose.pose.position.y = 7.55
    	goal.target_pose.pose.orientation.w = 0.711
    	goal.target_pose.pose.orientation.z= -0.703
    elif(mokuteke==2):
      goal.target_pose.pose.position.x = 0.46
      goal.target_pose.pose.position.y = -0.01
      goal.target_pose.pose.orientation.w = 1
      goal.target_pose.pose.orientation.z= 0
    elif(mokuteke==3):
      goal.target_pose.pose.position.x = 19.17
      goal.target_pose.pose.position.y = -0.73
      goal.target_pose.pose.orientation.w = 0.72
      goal.target_pose.pose.orientation.z= 0.694
    elif(mokuteke==4):
      goal.target_pose.pose.position.x = 19.29
      goal.target_pose.pose.position.y = 1.086
      goal.target_pose.pose.orientation.w = 0.72
      goal.target_pose.pose.orientation.z= 0.69
    else:
      goal.target_pose.pose.position.x = 19.17
      goal.target_pose.pose.position.y = -0.73
      goal.target_pose.pose.orientation.w = 0.72
      goal.target_pose.pose.orientation.z= 0.694

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        print("Action server not available!")
        return False
    else:
        return True
def absRotation(tarAngle, base_frame,rotateVel):
    global twistPub, angleTH,tfReq
    goal_frame='base_link'
    #base_frame = "charger"
    resp = tfReq(goal_frame , base_frame)
    quat = tuple(resp.quat)
    currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    #print("c",currYaw)
    # angleDiff is always smaller than 3.14, and used to judge whether the angle is reached with angleTH
    angleDiff =  abs(tarAngle - currYaw) if abs(tarAngle - currYaw) < 3.14 else (6.28 - abs(tarAngle - currYaw))
    ts = Twist()
    r = rospy.Rate(10)
    #print("rotate",angleDiff)
    while angleDiff> angleTH and not rospy.is_shutdown():
      # For a specific range, the robot should turn right
        if tarAngle - currYaw > 3.14 or ((tarAngle - currYaw) < 0 and (tarAngle - currYaw) > -3.14):
            ts.angular.z = -rotateVel # Turn right
        else:
            ts.angular.z = rotateVel  # Turn left
        twistPub.publish(ts)
        resp = tfReq(goal_frame, base_frame)
        quat = tuple(resp.quat)
        currYaw = tf.transformations.euler_from_quaternion(quat)[2]
    # Update angleDiff
        angleDiff =  abs(tarAngle - currYaw) if abs(tarAngle - currYaw) < 3.14 else (6.28 - abs(tarAngle - currYaw))
        r.sleep()

    ts.angular.z = 0.0
    twistPub.publish(ts)
    print("rotate")

def taskExeSrv(req):
  global mokuteke
  print("navi")
  mokuteke=req.goal_status
  if(mokuteke==3):
    outcome = NavStateMachine()
    mokuteke = 4  
  outcome = NavStateMachine()
  if(mokuteke==4):
    absRotation(1.57,"map",0.1)
  elif(mokuteke==5):
    absRotation(3.14,"map",0.25)

  return navigoalResponse(
      success= outcome,
      message="navigate executed!"
  )

if __name__ == '__main__':
  rospy.init_node('nav_state_machine')
  print("begin")
  cmdVelTopic=os.path.join('mobile_base_controller','cmd_vel')
  twistPub = rospy.Publisher(cmdVelTopic, Twist, queue_size=1)
  rospy.wait_for_service('robot_tf_server')
  tfReq = rospy.ServiceProxy('robot_tf_server', RobotTF)
  task_srv = rospy.Service('triggerNavigating', navigoal, taskExeSrv)
  rospy.spin()

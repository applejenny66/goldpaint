#!/usr/bin/env python

import actionlib
import rospy
import smach
import smach_ros
import os
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from ira_factory_msgs.srv import TaskExe, TaskExeResponse, UpdateRobotStatus
from ira_factory_msgs.msg import RobotStatus 
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathFeedback, ExePathResult
from mbf_msgs.msg import GetPathAction, GetPathGoal, GetPathResult
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from smach_ros import SimpleActionState
from pmc_navigation.srv import navigoal, navigoalResponse

controller = None
mokuteke = None


class ExePath(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'],
                               input_keys = ['path', 'controller'])
  def execute(self, userdata):
    # Check preemption
    if self.preempt_requested():
      self.service_preempt()
      return 'preempted'

    self.result = None
    self.feedback = None
    client = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    client.wait_for_server()
    exePathGoal = ExePathGoal()
    exePathGoal.path = userdata.path
    exePathGoal.controller = userdata.controller
    client.send_goal(exePathGoal, 
                     done_cb = self.doneCB,
                     feedback_cb = self.feedbackCB)

    # Monitoring feedback before get results
    # Once failure occur, return aborted
    rate = rospy.Rate(10)
    while self.result == None:
      if self.feedback != None:
        if self.feedback.outcome == 100:
          print('return aborted')
          return 'aborted'
      rate.sleep()

    return 'succeeded'

  def feedbackCB(self, feedback):
    self.feedback = feedback
    #rospy.loginfo('feedback = {}'.format(self.feedback.outcome))

  def doneCB(self, state, result):
    self.result = result


class Recovery(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes = ['succeeded', 'aborted', 'preempted'])

  def execute(self, userdata):
    # Check preemption
    if self.preempt_requested():
      self.service_preempt()
      return 'preempted'

    # Stop the robot
    client_stop = actionlib.SimpleActionClient('move_base_flex/exe_path', ExePathAction)
    client_stop.wait_for_server()
    exePathGoal = ExePathGoal()
    client_stop.send_goal(exePathGoal)

    client = actionlib.SimpleActionClient('move_base_flex/recovery', RecoveryAction)
    client.wait_for_server()


    recoveryGoal = RecoveryGoal()

    # First, moveback the robot
    rospy.loginfo('RECOVERY: Use moveback_recovery.')
    recoveryGoal.behavior = 'moveback_recovery'
    client.send_goal(recoveryGoal)
    client.wait_for_result()
    getRecoveryResult = client.get_result()
    if getRecoveryResult.outcome != 0:
      return 'aborted'

    # Second, clear costmap 
    recoveryGoal.behavior = 'clear_costmap'
    client.send_goal(recoveryGoal)
    client.wait_for_result()
    getRecoveryResult = client.get_result()
    if getRecoveryResult.outcome != 0:
      return 'aborted'

    return 'succeeded'


class NavStateMachine(smach.StateMachine):
  def __init__(self):
    global controller, mokuteke
    smach.StateMachine.__init__(self, outcomes = ['succeeded', 'aborted'])

    pss = PoseStamped()
    pss.header.frame_id = '/map'
    initial_pose = Pose()
    if(mokuteke==1):
    	initial_pose.position.x = -1.84
    	initial_pose.position.y = 7.45
    	initial_pose.position.z = 0
    	initial_pose.orientation.w = 0.69
    	initial_pose.orientation.z = -0.724
    elif(mokuteke==2):
     	initial_pose.position.x = 0
    	initial_pose.position.y = 3
    	initial_pose.position.z = 0
    	initial_pose.orientation.w = 0.707
    	initial_pose.orientation.z = 0.707
    elif(mokuteke==3):
     	initial_pose.position.x = 0
    	initial_pose.position.y = -0.56
    	initial_pose.position.z = 0
    	initial_pose.orientation.w = 1
    	initial_pose.orientation.z = 0
    elif(mokuteke==4):
     	initial_pose.position.x = 19.16
    	initial_pose.position.y = -0.275
    	initial_pose.position.z = 0
    	initial_pose.orientation.w = 0.724
    	initial_pose.orientation.z = 0.69
    elif(mokuteke==5):
     	initial_pose.position.x = 0
    	initial_pose.position.y = 3
    	initial_pose.position.z = 0
    	initial_pose.orientation.w = 0.707
    	initial_pose.orientation.z = -0.707			
    pss.pose = initial_pose 
    self.userdata.tarPS = pss
    self.userdata.path = None
    self.userdata.controller = controller

    with self:
      def pathCB(userdata, status, result):
        if status == GoalStatus.SUCCEEDED:
          userdata.path = result.path
      def exePathCB(userdata, status, result):
        print(status)

      smach.StateMachine.add('GENERATE_PATH',
                SimpleActionState('move_base_flex/get_path', GetPathAction, 
                              output_keys = ['path'],
                              goal_slots = ['target_pose'],
                              result_cb = pathCB),
                remapping = {'target_pose':'tarPS',
                             'path':'path'},
                transitions={'succeeded':'EXE_PATH',
                             'aborted':'aborted',
                             'preempted':'GENERATE_PATH'})

      smach.StateMachine.add('EXE_PATH', ExePath(),
                              transitions = {'succeeded':'succeeded', 
                                             'aborted':'RECOVERY',
                                             'preempted':'GENERATE_PATH'}) 

      '''
      smach.StateMachine.add('EXE_PATH',
                SimpleActionState('move_base_flex/exe_path', ExePathAction, 
                              input_keys = ['path'],
                              goal_slots = ['path', 'controller']),
                remapping = {'controller':'controller',
                             'path':'path'},
                transitions={'succeeded':'succeeded',
                             'aborted':'RECOVERY',
                             'preempted':'GENERATE_PATH'})
      '''

      smach.StateMachine.add('RECOVERY', Recovery(),
                              transitions = {'succeeded':'GENERATE_PATH',
                                             'aborted':'aborted',
                                             'preempted':'GENERATE_PATH'}) 

def taskExeSrv(req):
  global mokuteke
  print("navi")
  mokuteke=req.goal_status

  SM = NavStateMachine()
  SM.execute()
  #sis = smach_ros.IntrospectionServer('my_smach_introspection_server', SM, '/SM_ROOT')
  #sis.start()
  return navigoalResponse(
      success=True,
      message="navigate executed!"
  )

if __name__ == '__main__':
  rospy.init_node('nav_state_machine')
  print("begin")
  controller = rospy.get_param('~controller', 'dwa')
  task_srv = rospy.Service('navigation_execution', navigoal, taskExeSrv)
  rospy.spin()

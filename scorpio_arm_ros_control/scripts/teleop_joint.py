#!/usr/bin/env python
import rospy
import actionlib
import sys, select, termios, tty

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

PERIOD = 3 
POS_DIFF = 0.0010
TIME_DIFF = 0.05
g_pos_list = [None] * 6
g_jnt_dir = [1, 1, 1, 1, 1, 1] 
g_get_jnt_state = False
msg_hint="""

===================================
Specify which joint to control:
1: joint1
2: joint2
3: joint3
4: joint4
5: joint5
6: joint6
e: exit

Specify control type:
q: accelerate, j: forward 
z: decelerate, k: backward
e: exit
===================================
"""

def getKey():
  tty.setraw(sys.stdin.fileno())
  select.select([sys.stdin], [], [], 0)
  key = sys.stdin.read(1)
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  return key


def vels(speed,turn):
  return "currently:\tspeed %s\tturn %s " % (speed,turn)

def jnt_states_cb(data):
  global g_get_jnt_state
  if not g_get_jnt_state:
    for idx in xrange(6):
      g_pos_list[idx] = data.position[idx]
    print("\n===================================")
    print("Get initial joint positions:\n[{} {} {} {} {} {}]".format(g_pos_list[0], g_pos_list[1], g_pos_list[2], g_pos_list[3], g_pos_list[4], g_pos_list[5]))
    print("===================================")
    g_get_jnt_state = True
  

if __name__=="__main__":
  settings = termios.tcgetattr(sys.stdin)

  rospy.init_node('scorpio_arm_teleop')

  # Subscribe to topic
  jnt_states_sub = rospy.Subscriber("/joint_states", JointState, jnt_states_cb)

  # Subscribe to client
  client = actionlib.SimpleActionClient(
    'arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
  client.wait_for_server()


  pos_diff = POS_DIFF
  status = 0
  joint = 1

  # Wait for current joint state to come
  while(not g_get_jnt_state):
    print("Not get current joint positions yet...")
    continue

  print(msg_hint)

  while not rospy.is_shutdown():
    key = getKey()

    # Print hint
    if (status == 14):
      print(msg_hint)

    # change joint
    if key == '1':
      print("operate joint1")
      print
      joint = 1
      continue

    elif key == '2':
      print("operate joint2")
      print
      joint = 2
      continue

    elif key == '3':
      print("operate joint3")
      print
      joint = 3
      continue

    elif key == '4':
      print("operate joint4")
      print
      joint = 4

      continue

    elif key == '5':
      print("operate joint5")
      print
      joint = 5
      continue

    elif key == '6':
      print("operate joint6")
      print
      joint = 6
      continue

    # Forward
    elif key == 'j':
      direction = 1 

    # Backward
    elif key == 'k':
      direction = -1

    # Accelerate
    elif key == 'q':
      pos_diff *= 1.1 
      print("pos_diff = {}".format(pos_diff))
      status = (status + 1) % 15
      continue

    # Decelerate
    elif key == 'z':
      pos_diff *= 0.9 
      print("pos_diff = {}".format(pos_diff))
      status = (status + 1) % 15
      continue

    elif key == 'e':
      print("exit")
      break

    else:
      direction = 0
      continue


    # Creates a goal to send to the action server.
    ac_goal = FollowJointTrajectoryGoal()
    ac_goal.trajectory.header.frame_id = '/base_link'
    ac_goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    try:
      # Create a trajcetory for short period
      for idx in xrange(0, PERIOD):
        points = JointTrajectoryPoint()    
        points.positions = g_pos_list 

        points.positions[joint - 1] += direction * g_jnt_dir[joint - 1] *(idx * pos_diff) 
        #print(points.positions)
        #print(direction * idx * pos_diff)
        points.time_from_start =rospy.Duration(idx * TIME_DIFF)

        ac_goal.trajectory.points.append(points)
        '''
        for idx_temp in range(0, 6):
          print(points.positions[idx_temp])
        '''

    except TypeError:
      rospy.logwarn("Waiting for initial joint positions...")
      continue


    client.send_goal(ac_goal)
    client.wait_for_result()
    del ac_goal
    #result = client.get_result() 
    



  '''
  finally:
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
  '''


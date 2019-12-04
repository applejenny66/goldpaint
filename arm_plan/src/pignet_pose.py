#!/usr/bin/env python


import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import time

group= moveit_commander.MoveGroupCommander("arm")
ps = Pose()

def pigment_c():
    # blue
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    ps.position.z = 0.1 #TODO# #stay
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)

def pigment_m():
    # red
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    ps.position.z = 0.1 #TODO# #stay
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)

def pigment_y():
    # yellow
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    ps.position.z = 0.1 #TODO# #stay
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)

def pigment_k():
    # black
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    ps.position.z = 0.1 #TODO# #stay
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)

def pigment_w():
    # white
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    ps.position.z = 0.1 #TODO# #stay
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)

def palette_pos():
    ps.position.x = 0.9 #TODO#
    ps.position.y = 0.1 #TODO#
    ps.position.z = 0.1 #TODO# #stay
    ps.orientation = Quaternion(*quaternion_from_euler(1.57, 0.000, 0.00, 'syxz'))
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.02)
    ps.position.z = 0.1 #TODO# #dip
    group.set_pose_target(ps)
    group.go()
    time.sleep(0.1)
    i = 3
    while i > 0:
        ps.position.x = ps.position.x + 0.1
        group.set_pose_target(ps)
        group.go()
        ps.position.x = ps.position.x - 0.2
        group.set_pose_target(ps)
        group.go()
        ps.position.x = ps.position.x + 0.1
        group.set_pose_target(ps)
        group.go()
        i -= 1
    ps.position.z = 0.1 #TODO# #stay
    time.sleep(0.02)
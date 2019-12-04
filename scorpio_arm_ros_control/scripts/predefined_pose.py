#!/usr/bin/env python

import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler


group = moveit_commander.MoveGroupCommander("arm")

# Choose "straight" or "folded"
group.set_named_target("folded")
group.go()

  

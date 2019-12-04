#!/usr/bin/env python


import moveit_commander
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import time
import cv2
group= moveit_commander.MoveGroupCommander("arm")
ps = Pose()
#img = cv2.imread("./test.png")
#print ("img shape: ", img.shape) # 217, 403

def img_2_csv():
    img = cv2.imread("./test.png")
    # shape = (217, 403, 3) -> 21.7 cm, 40.3 cm, rgb 3 colors

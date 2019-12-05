#!/usr/bin/env python


#import moveit_commander
#from geometry_msgs.msg import Pose, Quaternion
#from tf.transformations import quaternion_from_euler
import time
import cv2
#group= moveit_commander.MoveGroupCommander("arm")
#ps = Pose()
#img = cv2.imread("./test.png")
#print ("img shape: ", img.shape) # 217, 403

def img_2_csv():
    img = cv2.imread("./test.png")
    # shape = (217, 403, 3) -> 21.7 cm, 40.3 cm, rgb 3 colors
    shape = img.shape
    new_shape = (int(shape[0]/10), int(shape[1]/10), 3)
    print (new_shape)
    new_img = np.zeros(new_shape)
    tmp_total_r = 0
    tmp_total_g = 0
    tmp_total_b = 0
    for i in range(0, new_shape[0]):
        for j in range(0, new_shape[1]):
            for w in range(0, 10):
                for h in range(0, 10):
                    tmp_total_r += img[i*10+w, j*10+h, 0]
                    tmp_total_g += img[i*10+w, j*10+h, 0]
                    tmp_total_b += img[i*10+w, j*10+h, 0]
            tmp_total_r = int(tmp_total_r / 100)
            tmp_total_g = int(tmp_total_g / 100)
            tmp_total_b = int(tmp_total_b / 100)
            #new_img[]
            

if __name__ == "__main__":
    img_2_csv()
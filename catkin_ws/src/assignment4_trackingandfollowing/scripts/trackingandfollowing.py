#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from time import time

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+150):int((height/2)+170)][1:int(width)]
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        mask = self.threshold_image(hsv)

        cx, cy = self.calculate_centroid(mask, height, width)

        twist_object = self.calculate_control_command(cx, width)

        self.moveTurtlebot3_object.move_robot(twist_object)
        
    def threshold_image(self, hsv):
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return mask

    def calculate_centroid(self, mask, height, width):
        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2
        return cx, cy
        
    def calculate_control_command(self, cx, width):
        global cxlast, lasttime
        twist_object = Twist()
        twist_object.linear.x = 0.22
        p = max(-0.15, min(0.15, ((width/2) - cx)/3000))
        d = max(-0.15, min(0.15, (cxlast-cx)/(time() - lasttime)*1.2*1e7))
        twist_object.angular.z = p+d
        cxlast = cx
        lasttime = time()
        return twist_object

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
    lasttime = 0
    cxlast = 0
    main()
``

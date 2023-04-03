#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
from PID import PID

rospy.init_node('line_following_node', anonymous=True)
# twist_object = Twist()
moveTurtlebot3_object = MoveTurtlebot3()
pid_controller = PID(P = 0.0015, I = 0.000, D = 0.0007)
pid_controller.clear()

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        self.twist_object = Twist()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)] #crop height keep same width
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only white
        #HSV = 10 deg, 2.7% sat and 87.1% value
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
        
        foundLine = True #assume true at start and if division by 0 occurs make it false

        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
            find_traj = True
        except ZeroDivisionError:
            cx, cy = width/2, height/2
            find_traj = False
            
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        if find_traj == True:
            err = -1 * (width/2 - cx)
            pid_controller.update(err)
            self.twist_object.linear.x = 0.1
            self.twist_object.angular.z = pid_controller.output
        else:
            #self.twist_object.angular.z = 0.2
            self.twist_object.linear.x = 0.1

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(self.twist_object.angular.z))
        # Make it start turning
        moveTurtlebot3_object.move_robot(self.twist_object)
        
    def clean_up(self):
        moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()



def main():
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        moveTurtlebot3_object.clean_class()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()

if __name__ == '__main__':
        main()


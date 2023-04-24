#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CompressedImage
from move_robot import MoveTurtlebot3
from PID import PID


class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback,
                                          queue_size=1)
        # self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.camera_callback,
        #                                   queue_size=1)
        self.moveTurtlebot3_object = MoveTurtlebot3()
        self.twist_object = Twist()
        self.pid_controller = PID(P=0.0012, I=0.001, D=0.0005)
        self.pid_controller.clear()

    def camera_callback(self, data):
        # We select bgr8 because it's the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we don't need
        height, width, channels = cv_image.shape
        crop_img = cv_image[int(height - 20):int(height)][1:int(width)]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)

        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
            find_traj = True
        except ZeroDivisionError:
            cx, cy = height / 2, width / 2
            find_traj = False

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        if find_traj == True:
            kp = 0.4
            self.twist_object.linear.x = 0.15
            self.twist_object.angular.z = -kp * ((cx - width / 2) / (width / 2))
        else:
            self.twist_object.linear.x = 0.1
            self.twist_object.angular.z = 0.0

        rospy.loginfo("ANGULAR VALUE SENT===>" + str(self.twist_object.angular.z))
        # Make it start turning
        self.moveTurtlebot3_object.move_robot(self.twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
    ctrl_c = False

    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()


if __name__ == '__main__':
    main()

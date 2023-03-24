#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3

class LineFollower:
    def __init__(self, camera_topic):
        # Initialize the ROS node
        rospy.init_node('line_follower_node')

        # Subscribe to the camera feed
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)

        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Create an instance of MoveTurtlebot3 to control the robot's movement
        self.robot = MoveTurtlebot3()

    def camera_callback(self, data):
        try:
            # Convert the ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Crop the image to focus only on the area where the line is expected
        height, width, channels = cv_image.shape
        roi = cv_image[int(height*0.5):height, int(width*0.25):int(width*0.75)]

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Threshold the image to extract the yellow line
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate the centroid of the yellow line
        M = cv2.moments(mask)
        if M['m00'] == 0:
            # The line is not detected
            return
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        # Draw a circle at the centroid of the yellow line
        cv2.circle(roi, (cx, cy), 10, (0, 0, 255), -1)

        # Calculate the error between the centroid and the center of the image
        err = cx - width/2

        # Create a Twist message to control the robot's movement
        twist = Twist()

        # Set the linear speed to a constant value
        twist.linear.x = 0.3

        # Set the angular speed proportional to the error
        twist.angular.z = -float(err) / 1000

        # Send the movement command to the robot
        self.robot.move_robot(twist)

        # Show the image with the line detected
        cv2.imshow('Line', roi)
        cv2.waitKey(1)

    def run(self):
        # Start the main loop
        rospy.spin()

    def shutdown(self):
        # Stop the robot's movement and destroy any OpenCV windows
        self.robot.stop_robot()
        cv2.destroyAllWindows()

if __name__ == '__main


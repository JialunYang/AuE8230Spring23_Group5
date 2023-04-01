#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

class AprilTagFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('april_tag_follower_node')

        # Subscribe to the AprilTag detections
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Create a CvBridge object to convert ROS images to OpenCV images
        self.bridge = CvBridge()

        # Create a Twist message to control the robot's movement
        self.twist = Twist()

    def tag_callback(self, data):
        if len(data.detections) == 0:
            # No tags detected
            return

        # Get the position and orientation of the first detected tag
        tag = data.detections[0]
        tag_id = tag.id[0]
        tag_position = tag.pose.pose.position
        tag_orientation = tag.pose.pose.orientation

        # Print the tag information
        rospy.loginfo('Detected tag %d at position (%.2f, %.2f, %.2f)' % (tag_id, tag_position.x, tag_position.y, tag_position.z))

        # Move the robot towards the tag
        self.twist.linear.x = 0.2
        self.twist.angular.z = -0.3 * tag_position.x
        self.cmd_vel_pub.publish(self.twist)

    def run(self):
        # Create a publisher for the Twist message
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Start the main loop
        rospy.spin()

    def shutdown(self):
        # Stop the robot's movement
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    follower = AprilTagFollower()
    follower.run()


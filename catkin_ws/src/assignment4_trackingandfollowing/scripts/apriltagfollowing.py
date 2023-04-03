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
        rospy.init_node('april_tag_follower_node', anonymous=True)
        
        # Publish to velocity control
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscribe to the AprilTag detections
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)

        # Other init
        self.rate = rospy.Rate(10)
        self.x = 0
        self.z = 0

    def tag_callback(self, data):
        tag = data.detections[0]
        tag_id = tag.id[0]
        self.x = tag.pose.pose.pose.position.x
        self.z = tag.pose.pose.pose.position.z
        self.y = tag.pose.pose.pose.position.y
               
        # Print the tag information
        rospy.loginfo('Detected tag %d at position (%.2f, %.2f, %.2f)' % (tag_id, self.x, self.y, self.z))

    def run(self):
        self.vel_msg = Twist()
        kx = 0.2
        kz = -2
        while not rospy.is_shutdown():
            # Control and publish the velocity
            if self.z >=0.4:
                self.vel_msg.linear.x = self.z*kx
                self.vel_msg.angular.z = self.x*kz
            elif self.z <= 0.3:
                self.vel_msg.linear.x = -self.z*kx
                self.vel_msg.angular.z = -self.x*kz
            else:
                self.vel_msg.linear.x = 0
                self.vel_msg.angular.z = 0
                                
            self.vel_pub.publish(self.vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    follower = AprilTagFollower()
    follower.run()


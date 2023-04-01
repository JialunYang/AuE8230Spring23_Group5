#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray


class AprilTagFollower():

    def __init__(self):
        self.x = 0
        self.z = 0
        self.k_linear_vel = 0.4
        self.k_angular_vel = 1
        # initializing a node
        rospy.init_node('AprilTagFollower', anonymous=True)
        # defining publisher
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # defining subscriber
        self.sub = rospy.Subscriber(
            '/tag_detections', AprilTagDetectionArray, self.pose)
        # subscriber -Apriltagdetection array callback function that stores the tag detection array coordinates in xyz world frame
        self.rate = rospy.Rate(10)

    def pose(self, data):
        # extracting pose of april tag
        self.x = data.detections[0].pose.pose.pose.position.x
        self.z = data.detections[0].pose.pose.pose.position.z
        print("The front distance is :", self.x)
        print("The header is :", self.z)

    def april_tag_follower(self):
        self.vel_msg = Twist()
        while not rospy.is_shutdown():
            # velocity controller
            self.vel_msg.linear.x = self.z*self.k_linear_vel
            self.vel_msg.angular.z = -self.x*self.k_angular_vel
            # publishing velocity
            self.pub.publish(self.vel_msg)
            self.rate.sleep()


if __name__ == '__main__':
    tagfollow = AprilTagFollower()
    tagfollow.april_tag_follower()

#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scan_callback(data):
    scan_value = data.ranges[len(data.ranges)//2] # get the center-most value of the scan data array
    threshold = 0.5 # define the distance threshold for emergency braking

    if scan_value < threshold:
        move.linear.x = 0 # publish 0 to /cmd_vel if the robot gets too close to the wall
        move.angular.z = 0
    else:
        move.linear.x = 0.2     # High Speed
        #move.linear.x = 0.4    # Medium Speed
        #move.linear.x = 0.6    # High Speed
        move.angular.z = 0

    pub.publish(move)

rospy.init_node('emergency_braking')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
move = Twist()

rospy.spin()


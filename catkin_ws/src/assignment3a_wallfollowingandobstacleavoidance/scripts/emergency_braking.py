#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def scan_callback(data):
    # scan_value = data.ranges[len(data.ranges)//2] # get the center-most value of the scan data array
    scan_value = data.ranges[0]
    print('Range at 0 degress: {}'.format(data.ranges[0]))
    threshold = 0.5 # The distance threshold for emergency braking

    if scan_value < threshold:
        move.linear.x = 0 # publish 0 to /cmd_vel if the robot gets too close to the wall
        move.angular.z = 0
    else:
        move.linear.x = 0.2     # Low Speed
        #move.linear.x = 0.4    # Medium Speed
        #move.linear.x = 0.6    # High Speed
        move.angular.z = 0

    pub.publish(move)

rospy.init_node('emergency_braking')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
move = Twist()

rospy.spin()

if __name__ == '__main__':
    eb = scan_callback()
    eb.run()

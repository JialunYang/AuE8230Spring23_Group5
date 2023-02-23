#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
@file: circle.py 
@description: 
"""

import rospy
from geometry_msgs.msg import Twist

def circle():
    rospy.init_node('circle', anonymous = True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    rate =rospy.Rate(1)

    while not rospy.is_shutdown():
        #vel_msg.linear.x = 0.8    #medium
        #vel_msg.linear.x = 0.3    #slow
        vel_msg.linear.x = 1.5    #fast
        #vel_msg.angular.z = 1.5   #medium
        #vel_msg.angular.z = 1.0   #slow
        vel_msg.angular.z = 1.8   #fast

        velocity_publisher.publish(vel_msg)

        rate.sleep()

    # If we press ctrl + C, the node will stop.
    rospy.spin()

if __name__ == '__main__':
    try:
        # Testing our function
        circle()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from math import pow, atan2, sqrt, pi

def square_openloop():
    # Starts a new node
    rospy.init_node('square_openloop', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    while not rospy.is_shutdown():
        # setting speed and distance for straight line walking
        vel_msg.linear.x = 0.3    #medium
        #vel_msg.linear.x = 0.2    #slow
        #vel_msg.linear.x = 0.4    #fast
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        speed = 0.3
        distance = 2

        # Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        # Loop to move the turtle in an specified distance
        while (current_distance < distance):
            # Publish the velocity
            velocity_publisher.publish(vel_msg)
            # Takes actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # Calculates distancePoseStamped
            current_distance = speed * (t1 - t0)

        # stop going forward
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)

        # setting speed and steering angle for rotating
        vel_msg.angular.z = 0.3    #medium
        #vel_msg.angular.z = 0.2    #slow
        #vel_msg.angular.z = 0.4    #fast
        angular_speed = 0.3
        relative_angle = pi / 2

        # Setting the current time for distance calculus
        t2 = rospy.Time.now().to_sec()
        current_angle = 0

        # Loop to rotate for 90 degree
        while(current_angle < relative_angle):
            velocity_publisher.publish(vel_msg)
            t3 = rospy.Time.now().to_sec()
            current_angle = angular_speed * (t3 - t2)
    
        # Stop rotating
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        # Testing our function
        square_openloop()
    except rospy.ROSInterruptException:
        pass

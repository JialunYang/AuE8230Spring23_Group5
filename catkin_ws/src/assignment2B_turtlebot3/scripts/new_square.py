#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
from math import pi

def move():
    # Starts a new node
    rospy.init_node('turtlebot3', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's move your robot")

    speed = 0.3    #medium
    #speed = 0.2    #slow
    #speed = 0.4    #fast
    distance = 0.6
    
    angular_speed = 0.3    #medium
    #angular_speed = 0.2    #slow
    #angular_speed = 0.4    #fast
    angle = (90*pi)/180

    while not rospy.is_shutdown():

        n = 4
        current_distance = 0
        current_angle = 0
        while n != 0:

            t1 = rospy.Time.now().to_sec()
            

            while current_distance < distance and n > 0:

              vel_msg.linear.x = speed
              vel_msg.angular.z = 0
              velocity_publisher.publish(vel_msg)
              t2=rospy.Time.now().to_sec()
              current_distance= speed*(t2-t1)

              t3 = rospy.Time.now().to_sec()
            current_distance = 0   

            while current_angle < angle and n > 0:
                           
              vel_msg.linear.x = 0
              vel_msg.angular.z = abs(angular_speed)
              velocity_publisher.publish(vel_msg)
              t4 = rospy.Time.now().to_sec()
              current_angle = angular_speed*(t4-t3)
            current_angle = 0 
            n = n - 1

        #After the loop, stops the robot
    vel_msg.linear = 0
    vel_msg.angular = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass

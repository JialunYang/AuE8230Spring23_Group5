#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np


def PID(error, Kp=0.1):
    return Kp * error


class Obstacle_Avoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)
        self.move = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.avoid_obstacle)
        # self.angular_vel = 0
        rospy.sleep(2)

    def avoid_obstacle(self, data):
        left = min(3.5, np.mean(data.ranges[270:340]))
        right = min(3.5, np.mean(data.ranges[20:90]))
        error = -(left-right)
        angular_vel = PID(error, 0.5)
        self.move.linear.x = 0.2
        self.move.angular.z = angular_vel

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.move)


if __name__ == '__main__':
    oa = Obstacle_Avoidance()
    oa.run()


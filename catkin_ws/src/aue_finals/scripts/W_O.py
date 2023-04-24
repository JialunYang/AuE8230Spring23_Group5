#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def PID(error, Kp=0.1):
    return Kp * error


class WFAndOA:
    def __init__(self):
        rospy.init_node('wf_oa')
        self.move = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.follow_wall)
        self.switch = False

    def follow_wall(self, data):
        if not self.switch:
            print('mode 1')
            lidar_scan = list(data.ranges[0:359])
            scan = [x for x in lidar_scan if x < 3]
            # right = sum(scan[-90:-16]) / 74
            # left = sum(scan[16:90]) / 74
            right = sum(scan[-90:-20]) / len(scan[-90:-20])
            left = sum(scan[20:90]) / len(scan[20:90])

            front_cone = []
            for p in range(len(data.ranges)):
                if p < 20 or p > 340:
                    front_cone.append(data.ranges[p])
            front_min = min(front_cone)

            error = left - right
            self.move.linear.x = 0.2
            self.move.angular.z = PID(error, 0.9)
            print("Angular Velocity is %s" % self.move.angular.z)
            if front_min < 0.3:
                rospy.loginfo("Front obstacle detected" + str(front_min))
                # print(front_min)
                self.switch = True
                self.move.linear.x = 0
                self.move.angular.z = 0
        else:
            print('mode 2')
            left = min(3.5, np.mean(data.ranges[270:340]))
            right = min(3.5, np.mean(data.ranges[20:90]))
            error = -(left - right)
            self.move.linear.x = 0.1
            self.move.angular.z = PID(error, 0.3)
            print("Angular Velocity is %s" % self.move.angular.z)

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.move)


if __name__ == "__main__":
    w_o = WFAndOA()
    w_o.run()

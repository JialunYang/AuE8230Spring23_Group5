#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollowing:
    def __init__(self):
        self.move = Twist()
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.wallfollow)
        self.thresh = 0.6

    def wallfollow(self, data):
        print('Range at 20 degress: {}'.format(data.ranges[20]))
        print('Range at 55 degress: {}'.format(data.ranges[55]))
        print('Range at 90 degress: {}'.format(data.ranges[90]))
        print('Range at 270 degress: {}'.format(data.ranges[270]))
        print('Range at 305 degress: {}'.format(data.ranges[305]))
        print('Range at 340 degress: {}'.format(data.ranges[340]))

        if (data.ranges[20] > self.thresh and 
            data.ranges[340] > self.thresh and 
            data.ranges[0] > self.thresh):
            self.move.linear.x = 0.5
            self.move.angular.z = 0
        elif (data.ranges[20] > self.thresh and 
              data.ranges[55] > self.thresh and 
              data.ranges[90] > self.thresh):
            self.move.linear.x = 0.0
            self.move.angular.z = 0.15
            if (data.ranges[0] > self.thresh and 
                data.ranges[20] > self.thresh and 
                data.ranges[340] > self.thresh):
                self.move.linear.x = 0.5
                self.move.angular.z = 0.0
        elif (data.ranges[270] > self.thresh and 
              data.ranges[305] > self.thresh and 
              data.ranges[340] > self.thresh):
            self.move.linear.x = 0.0
            self.move.angular.z = -0.15
            if (data.ranges[0] > self.thresh and 
                data.ranges[20] > self.thresh and 
                data.ranges[340] > self.thresh):
                self.move.linear.x = 0.5
                self.move.angular.z = 0.0
        else:
            self.move.linear.x = 0
            self.move.angular.z = 0.5

        self.pub.publish(self.move)

    def run(self):
        rospy.init_node('wall_following')
        rospy.spin()

if __name__ == '__main__':
    wf = WallFollowing()
    wf.run()

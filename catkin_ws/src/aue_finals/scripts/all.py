#!/usr/bin/env python3

from pynput import keyboard
# from apriltag_ros.msg import AprilTagDetectionArray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CompressedImage
from darknet_ros_msgs.msg import BoundingBoxes
import math
import numpy as np
from move_robot import MoveTurtlebot3


def PID(error, Kp=0.1):
    return Kp * error


def on_press(key):
    pass


def on_release(key):
    # key is pynput.keyboard.KeyCode for letters and numbers
    # INITIAL STATE IS 0!

    # to tell Python we are editing a global var inside a function need to write "global subscriber"
    global subscriber  # only need this ONCE
    global subscriber2  # need two because line following and stop sign task needs to subscribe to two topics

    # initial state, don't move
    if key.char == "0":
        state = 0
        print("State " + str(state) + " activated")

        subscriber.unregister()
        subscriber2.unregister()
        cv2.destroyAllWindows()

        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()

        subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)
        subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)

    # State 1: Wall Following and Obstacle Avoidance
    elif key.char == "1":
        state = 1
        print("State " + str(state) + " activated")

        subscriber.unregister()
        subscriber2.unregister()
        cv2.destroyAllWindows()

        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()

        subscriber = rospy.Subscriber("/scan", LaserScan, wallFollow_Callback)
        subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)

    # State 2 = Line Following and Stop Sign
    elif key.char == "2":
        state = 2
        print("State " + str(state) + " activated")

        subscriber.unregister()
        subscriber2.unregister()
        cv2.destroyAllWindows()

        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()

        # line_follower_object = LineFollower() #creates a cv bridge
        subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, camera_callback, queue_size=1)
        subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.yolo_callback)

    # State 3 = April Tag
    elif key.char == "3":
        state = 3
        print("State " + str(state) + " activated")

        subscriber.unregister()
        subscriber2.unregister()
        cv2.destroyAllWindows()

        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()

        subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_update)
        subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)


def empty_callback(msg):
    pass


def wallFollow_Callback(msg):
    print("entered wallFollow callback")
    if not self.switch:
        print('mode 1')
        lidar_scan = list(data.ranges[0:359])
        scan = [x for x in lidar_scan if x < 3]
        right = sum(scan[-90:-20]) / len(scan[-90:-20])
        left = sum(scan[20:90]) / len(scan[20:90])

        front_cone = []
        for p in range(len(data.ranges)):
            if p < 20 or p > 340:
                front_cone.append(data.ranges[p])
        front_min = min(front_cone)

        error = left - right
        vel.linear.x = 0.2
        vel.angular.z = PID(error, 0.9)
        print("Angular Velocity is %s" % vel.angular.z)
        if front_min < 0.3:
            rospy.loginfo("Front obstacle detected" + str(front_min))
            # print(front_min)
            self.switch = True
            vel.linear.x = 0
            vel.angular.z = 0
        publisher.publish(vel)
        rate.sleep()
        return
    else:
        print('mode 2')
        left = min(3.5, np.mean(data.ranges[270:340]))
        right = min(3.5, np.mean(data.ranges[20:90]))
        error = -(left - right)
        vel.linear.x = 0.1
        vel.angular.z = PID(error, 0.3)
        print("Angular Velocity is %s" % vel.angular.z)
        publisher.publish(vel)
        rate.sleep()
        return


# LINE FOLLOW AND STOP SIGN CODE
def camera_callback(data):
    # We select bgr8 because it's the OpneCV encoding by default
    cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

    # We get image dimensions and crop the parts of the image we don't need
    height, width, channels = cv_image.shape
    crop_img = cv_image[int(height - 20):int(height)][1:int(width)]

    # Convert from RGB to HSV
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    # Define the Yellow Colour in HSV

    """To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and 
    convert to HSV."""

    # Threshold the HSV image to get only yellow colors
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([50, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Calculate centroid of the blob of binary image using ImageMoments
    m = cv2.moments(mask, False)

    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        find_traj = True
    except ZeroDivisionError:
        cx, cy = height / 2, width / 2
        find_traj = False

    # Draw the centroid in the resultut image
    # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]])
    # cv2.circle(mask, (int(cx), int(cy)), 10, (0, 0, 255), -1)
    # cv2.imshow("Original", cv_image)
    # cv2.imshow("MASK", mask)
    # cv2.waitKey(1)

    #################################
    ###   ENTER CONTROLLER HERE   ###
    #################################
    if find_traj:
        error = -((cx - width / 2) / (width / 2))
        vel.linear.x = 0.15
        vel.angular.z = PID(error, 0.4)
    else:
        vel.linear.x = 0.1
        vel.angular.z = 0.0

    rospy.loginfo("ANGULAR VALUE SENT===>" + str(self.twist_object.angular.z))
    # Make it start turning
    # publisher.publish(vel)
    moveTurtlebot3_object.move_robot(vel)
    rate.sleep()


def yolo_callback(msg):
    # global detectedStopSign
    # if not detectedStopSign:
    #     for i in msg.bounding_boxes:
    #         if i.Class == "stop sign":
    #             rospy.loginfo('Stop sign detected! Stop for 3 seconds.')
    #             detectedStopSign = True
    #             vel.angular.z = 0
    #             vel.linear.x = 0
    #             moveTurtlebot3_object.cmd_vel_subs.unregister()
    #             rospy.sleep(3.)
    #             vel.linear.x = 0
    #             vel.angular.z = -0.5
    #             moveTurtlebot3_object.cmd_vel_subs = rospy.Subscriber('/cmd_vel', Twist, self.moveTurtlebot3_object.cmdvel_callback)
    pass

def tag_callback(msg):
    pass

rospy.init_node('turtlesim_controller', anonymous=True)

# global variables (need to be used in multiple functions)
state = 0

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # all states will use this so put it here for all to use
vel = Twist()  # initialize vel as a Twist message
rate = rospy.Rate(40)
moveTurtlebot3_object = MoveTurtlebot3()

subscriber = rospy.Subscriber("/scan", LaserScan, empty_callback)
subscriber2 = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, empty_callback)

bridge_object = CvBridge()
detectedStopSign = False
switch = False

x = 0
z = 0

if __name__ == '__main__':
    try:
        # ensure the robot is not moving at the start
        vel.linear.x = 0
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = 0

        publisher.publish(vel)
        rate.sleep()

        print("Initial state = " + str(state))

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        # code won't run unless this while loop is present, so have it with a pass
        while not rospy.is_shutdown():
            pass

    except rospy.ROSInterruptException:
        pass

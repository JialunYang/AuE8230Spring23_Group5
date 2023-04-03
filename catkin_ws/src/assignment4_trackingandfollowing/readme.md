Overview:
For the purpose of implementing line-following behavior on a Turtlebot3 robot using ROS, this repository offers Python scripts and launch files. Robots must exhibit the basic behavior of line following in order to independently travel along a predetermined path. Using a camera and a PID controller, the robot monitors a yellow line and modifies its angular velocity to stay on course.

 An AprilTag detection script is also included in the code, allowing the robot to find and locate AprilTags. 

Part1: Line following (Gazebo and Real-world)

A Python script for line following on a Turtlebot3 robot using ROS is available in this repository. The line following.py script tracks a yellow line with OpenCV and determines its centroid. The script crops the picture to focus on the region where the line is anticipated to be by subscribing to the image subject from the camera. The script then uses a color threshold to isolate the yellow color in the picture, and after that, it uses image moments to determine the centroid of the resulting binary image. Based on the inaccuracy between the centroid of the line and the image center, a PID controller determines the robot's rotational velocity. The robot spins according to the output of the PID controller and advances at a constant linear speed.

 Run the following command in a simulation environment to launch the line following (Gazebo) behavior.

    $ roslaunch assignment4 follow_line_hsv2.launch

Run the following command in a simulation environment to launch the line following(Real-World) behavior.  

    $ rosrun assignment4_trackingandfollowing follow_line_step_hsv_real.py

Troubleshooting(Real-world):

Uneven Surface: An uneven or bumpy surface may interfere with sensor readings and lead the robot to deviate from the path.
    • Line-following sensors are made to follow straight lines, not curved ones. The sensors may have problems if the line is curvy or makes abrupt turns.
Poor lighting conditions: Line following sensors require sufficient lighting to detect the contrast between the line and the surface. Poor lighting conditions can result in inaccurate readings or no readings at all.



Part2: April Tag Tracking

The code establishes a publisher for the Twist messages and a subscriber for the AprilTagDetectionArray messages as well as initializes a ROS node called "AprilTagFollower." The "pose" callback function of the subscriber retrieves the posture of the discovered AprilTag and stores the x, y, and z coordinates of that position in the self.x and self.z variables. As a velocity controller, the "April tag follower" method of the AprilTagFollower class sets the linear and angular velocities depending on the self.x and self.z values and publishes them using the publisher. The velocity messages are broadcast at a set rate thanks to the rospy.Rate function. The April tag used by the code is Tag36h11.

Run the following command in a simulation environment to launch the line following(April tag tracking) behaviour.  

    $ rosrun assignment4_trackingandfollowing apriltagfollowing.py


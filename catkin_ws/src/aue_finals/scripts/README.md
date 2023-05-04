The AuE 8230 Autonomy: Science and Systems course provided hands-on experience with gazebo simulation and implementing various autonomous tasks in the real world. The project included the following tasks:
    • Wall following
    • Obstacle avoidance
    • Line following
    • Stop sign detection
    • April tags detection
The main challenge in the project was integrating all the tasks and switching between them to enable the TurtleBot 3 Burger to operate autonomously both in the gazebo environment and in the real world.
In the gazebo environment, the TurtleBot3 Burger was made to navigate through the wall following task and then avoid the obstacles placed in between. After that, the bot also had to follow the yellowish green line, which was yellow in the gazebo environment. As the bot navigated through the yellow line, it had to detect the stop sign and come to a halt for three seconds. Finally, after detecting the stop sign, the TurtleBot3 Burger had to detect and follow the April tag placed near the end of the yellow line.
In the real world, the obstacles were changed each time by the Teaching Assistants for obstacle avoidance. The TurtleBot3 Burger had to follow the same yellow line and detect the same stop sign and April tag as in the gazebo environment.

    For real-world environment: rosrun aue_finals all_real.py
    For Gazebo environment: roslaunch aue_finals all.launch


Functionality:

The main all_real.py, script contains five different states of operation, which can be activated by pressing different keys on the keyboard:

    State 0: initial state, where the robot does not move.
    State 1: Wall Following and Obstacle Avoidance
    State 2: Line Following and Stop Sign Detection
    State 3: April Tag Detection

State 1:
In state 1, the robot follows a wall while avoiding obstacles. This state is activated by pressing the key "1" on the keyboard. When the robot is in this state, it subscribes to the /scan topic to get laser scan data and uses it to perform wall following. The robot also checks for obstacles in front of it by looking at the minimum distance in front of the robot. If an obstacle is detected, the robot switches to mode 2.

State 2:
In state 2, the robot follows a line and stops at a stop sign. This state is activated by pressing the key "2" on the keyboard. When the robot is in this state, it subscribes to the /camera/image topic to get image data from the camera and uses it to perform line following. The robot also subscribes to the /darknet_ros/bounding_boxes topic to detect stop signs. If a stop sign is detected, the robot stops for 3 seconds and then continues with line following.

State 3:
In state 3, the robot follows an April tag. This state is activated by pressing the key "3" on the keyboard. When the robot is in this state, it subscribes to the /tag_detections topic to get April tag data and uses it to perform tag following. The robot moves towards the tag by adjusting its linear and angular velocities.

Dependencies

    pynput
    apriltag_ros
    cv2
    rospy
    matplotlib
    geometry_msgs
    sensor_msgs
    darknet_ros_msgs
    numpy
    PID
    
Our project successfully implemented various autonomous tasks using the TurtleBot3 robot, such as wall following, obstacle detection, line following, stop sign detection, and April tag detection autonomously. However, we faced technical challenges specific to our project, such as issues with mode switching, detecting obstacles with varying widths, and tuning speed parameters. We also learned the importance of effective planning and execution when integrating modules to avoid potential issues. Overall, the project provided practical application of course concepts, and we gained valuable experience and skills for future autonomy work.

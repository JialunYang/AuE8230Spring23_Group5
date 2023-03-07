Overview:

This repository contains Python scripts and launch files for implementing obstacle avoidance and wall following behaviors on a Turtlebot3 robot using ROS. These behaviors are commonly used in robotics applications and are essential for robots to navigate through complex environments without colliding with obstacles.                       
 
Part 1: Wall Following

The wall_following.py Python script in the wall following package implements the wall following behavior.
The script subscribes to the scan topic in order to obtain range data from the robot's laser scanner.
We used A PID Controller is used to control the motion of the robot by taking /scan data as input from the left and right sides of the turtlebot3. Kp value is iterated between 0.4 to 1.0 is used with conjuction to a linear velocity of 0.45. Here the error is estimated scanning by subtracting left data with right data and the error is fed to PID controller and which helped in correcting the angular velocity.
The robot moves in the direction with the greatest mean range value (i.e., the sector with the most open space).

Run the following command in a simulation environment to launch the wall following behavior: 

    • $ roslaunch assignment3a_wallfollowingandobstacleavoidance wall_follower.launch

Part 2: Obstacle Avoidance

The Python script obstacle_avoidance.py, which implements the obstacle avoidance behavior, is included in the obstacle avoidance package.
Using laser scanner data, this software is meant to make a robot follow a wall.
The robot advances until it discovers a wall, at which point it follows it while keeping a specified distance from it.
    • The variable thresh, which is initially set at 0.25, determines the minimum distance. 
    • If there is no wall in sight, the robot goes forward with a linear velocity of 0.4 and an angular velocity of 0.
    • If the robot detects a wall to its right, it rotates left with an angular velocity of 1.0 to follow it.
    • If the robot detects a wall to its left, it rotates right with an angular velocity of -1.0 to follow it.
    • If there is a wall ahead, the robot will stop traveling and turn until there is no wall to the right or left, at which point it will proceed forward with a linear velocity of 0.4 and an angular velocity of 0.0. 

Run the following command in a simulation environment to launch the obstacle avoidance behavior: 

    • $ roslaunch assignment3a_wallfollowingandobstacleavoidance wander.launch

Part 3: Obstacle Avoidance in Real world


Obstacle avoidance can also be run on a physical Turtlebot3 robot using the obstacle avoidance package.
To accomplish this, modify the obstacle_real.launch file to use a real robot rather than a simulated one.
To start the Turtlebot3 driver, you must also connect to the robot via SSH and run the following command: 

    • $ roslaunch assignment3a_wallfollowingandobstacleavoidance obstacles_real.launch

The obstacle avoidance can then be launched.
When you run the file, the robot will move around and avoid obstacles. 


Part 4: Emergency Braking

The emergency braking.py function subscribes to the '/scan' subject in order to obtain laser scan data and publishes to the '/cmd vel' topic in order to direct the robot's movement.
The script calculates the robot's distance from the nearest obstacle using the scan data array's center-most value.
If the distance is less than 0.5 meters, the robot will cease moving by posting 0 to the '/cmd vel' topic.
If the distance exceeds or equals the threshold value, the robot travels at a slow rate of 0.2 m/s by posting to the '/cmd vel' topic. 

Run the following command in a simulation environment to launch the Emergency Braking:

    • $ roslaunch assignment3a_wallfollowingandobstacleavoidance emergency_braking.launch

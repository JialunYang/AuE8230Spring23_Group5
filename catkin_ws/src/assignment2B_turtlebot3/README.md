$ roslaunch assignment2B_turtlebot3 move.launch code:=circle 

$ roslaunch assignment2B_turtlebot3 move.launch code:=square

These commands are used to launch the circle.py and square.py for our assignment.

Introduction:

The turtle bot is programmed to move and spin at various rates to move in square and circle. Here, we have made various observation on the trajectory it followed, movement of the turtle Bot before calibrating it. 
The lower medium and high speeds were taken as 0., 1.5, 1.8 for circle and  0.2, 0.3, and 0.4 for square respectively to get the results.


Methodology and observations: 

Circle:
We ran several experiments on the turtle bot, watching its movement and rotation at various speeds. We deployed a camera to capture its movement and then carefully studied the film to figure out what was wrong. We also examined the hardware and software components to confirm that they were in good working order. Unfortunately, during testing, we discovered problems with its spinning at medium and high speeds. The purpose of this report is to investigate and identify potential causes of the problem.

Square:
A square of 1 m is measured on the ground and the turtle bot is placed to estimate its trajectory and deviation from actual path. During low speeds the turtlebot has traced its path with little deviation but at medium speeds the orthogonal shape of the square got deviated diagonally. Whereas at high speeds the last side of the square shape got deviated.


Results:

Circle:
The turtle bot turned well and without problem at low speeds. As we increased the rotational speed to medium, we saw jerks and abrupt shifts in direction. Also the bot cannot go well when the speed of z is lower than 0.2, no matter what the direct speed is. So it has a minimum vertical speed.
Square:
The speed at which the turtlebot moves affects its ability to maintain a consistent trajectory. The slow speed was the most accurate, while the fast speed was the least accurate due to its high angular speed. It is also important to note that the surface on which the turtlebot travels can also impact its trajectory.  This emphasizes the importance of testing in real-world environments to obtain accurate results. Also for running in 1 meter, different speed need different "distance" parameters in the code. To be specific, the higher the speed is, the higher the distance need to be for the bot to run in 1 meter.


Circle radius in m - Deviation 

Slow- No deviation,
Medium- 30% reduction in radius,
Fast- 50% reduction in radius

Square edge in m - Deviation 

Slow- 0.01m offset,
Medium- 0.2m offset,
Fast- 0.25m offset


Initially, after completion of two test in circle the ip address of the turtle Bot got changed and we had to again change the HOST IP twice and continued to test the other manuvers.

Following a thorough investigation, we determined the following causes of the problems:

Possible case of error:

Mechanical Issues: Mechanical difficulties with the wheels or the motor might be the cause of the jerks and unexpected changes in direction during rotation. The wheels might be spinning unevenly, or the engine could be overheating, leading the turtle bot to lose control.

Control Algorithm: Another possible cause is a problem with the control algorithm. At greater speeds, the control algorithm may be unable to keep up with the turtle bot's pace, leading it to lose control.

Power Supply: A faulty or insufficient power supply may possibly be contributing to the rotation problems. The jerks might be caused by the motor not receiving enough power to work properly and also the battery got drained very often.

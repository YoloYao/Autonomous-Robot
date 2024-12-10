# Autonomous-Robot
Path planning robot based on turtlebot3

## Minitask1
Control the movement of the robot and walk out of the square path with a side length of 1 meter.
|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch turtlebot3_gazebo turtlebot3_world.launch|
|3|T3|rosrun minitask1 square_move_controller.py|

## Minitask2
1 After going straight for 1 meter, turn left and right at will and continue walking for 1 meter at random;

2 After detecting the wall, go straight along the right wall;

3  When obstacles are detected in front of you, rotate to avoid obstacles

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch minitask3 turtlebot3_corridors.launch|
|3|T3|rosrun minitask2 obstacle_avoidance.py|

## Minitask3
Use the robot camera to look for the green object in front of you, move towards the green object, and stop in advance to the target object. Avoid obstacles encountered during exercise
|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch minitask3 turtlebot3_corridors.launch|
|3|T3|rosrun minitask3 follower.py|
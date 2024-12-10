# Autonomous-Robot
Path planning robot based on turtlebot3

## Minitask1
Control the movement of the robot and walk out of the square path with a side length of 1 meter.
|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch turtlebot3_gazebo turtlebot3_world.launch|
|3|T3|rosrun minitask1 square_move_controller.py|
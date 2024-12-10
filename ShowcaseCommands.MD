<h1>Showcase Commands</h1>

<h2>Connect Robot</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|ssh team@turtlebot|
|2|T1|Fohvegh9|
|3|T1|cd catkin_ws|
|4|T1|source devel/setup.bash|
|5|T1|roslaunch turtlebot3_bringup turtlebot3_robot.launch|
|6|T2|roslaunch realsense2_camera rs_camera.launch align_depth:=true|


<h2>Minitask1</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch turtlebot3_gazebo turtlebot3_world.launch|
|3|T3|rosrun minitask1 square_move_controller.py|

<h2>Minitask2</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch minitask3 turtlebot3_corridors.launch|
|3|T3|rosrun minitask2 obstacle_avoidance.py|

<h2>Minitask3</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch minitask3 turtlebot3_corridors.launch|
|3|T3|rosrun minitask3 follower.py|

<h2>Minitask4</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch turtlebot3_gazebo turtlebot3_house.launch|
|3|T3|roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=\`rospack find minitask4\`/maps/realmap.yaml|
|4|T4|roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch|
|5|T5|rosrun minitask4 patrol.py|

<h2>Minitask5</h2>

|Step|Terminal|Commands|
|---|---|---|
|1|T1|roscore|
|2|T2|roslaunch minitask5 train_env.launch|
|3|T3|roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=\`rospack find minitask5\`/maps/train_env.yaml|
|4|T4|rosrun minitask5 main.py|




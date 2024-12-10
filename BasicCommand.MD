<h1>ROS Basic Commands</h1>

|Num|Function|Commands|
|---|---|---|
|1|create package|catkin_create_pkg minitask1 std_msgs geometry_msgs rospy|
|2|config|source ~/catkin_ws/devel/setup.bash|
|3|start|roscore|
|4|start world|roslaunch turtlebot3_gazebo turtlebot3_world.launch|
|5|start control by hand|roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch|
|6|show move info|rostopic echo /cmd_vel|
|7|show rviz interface|roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch|
|8|start corridors world|roslaunch minitask3 turtlebot3_corridors.launch|
|9|reset world|rosservice call gazebo/reset_world|
|10|start house environment|roslaunch turtlebot3_gazebo turtlebot3_house.launch|
|11|add the map visualisation|roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping|
|12|save map|rosrun map_server map_saver -f mymap|
|13|launch the navigation file|roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:='rospack find minitask4'/maps/mymap.yaml|
|14|start train world|roslaunch minitask5 train_env.launch|
|15|find coordinate in rviz|rostopic echo /clicked_point|
|||

<h1>Git Basic Commands</h1>

|Step|Commands|
|---|---|
|1|cd XXX|
|2|git init|
|3|git remote add origin https://projects.cs.nott.ac.uk/comp4034-ars-2024/g08.git|
|4|git add XXX|
|5|git status|
|6|git pull origin main|
|7|git commit -m “XXXXX”|
|8|git push origin master| 
|||

<h1>Developer Tokens</h1>

|Name|ID|Tokens|
|---|---|---|
|Yue YAO|alyyy85|glpat-7ux3PhbjGrLJ26Zne1mJ|
|Yinghao HE|alyyh76|glpat-_dkBs3zLKaK6XjwfZVU3|
|Xianda XU|alyxx35|glpat-p_XcY1zAmVksscCejk8D|
|Robot|-|Fohvegh9|


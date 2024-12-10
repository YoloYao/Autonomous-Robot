一、基本功能要求：
1.设定目标坐标（绿色框，红色消防栓），向几个目标依次移动
2.移动到目标前时停车，计算找到的物体数量并打印
3.行进过程中躲避蓝色地砖
4.行进过程中躲避前方障碍物
5.在 RViz 中在地图上标记发现障碍物的位置

二、其他要求：
1.使用 Lidar 和 Kinect 传感器（在 RGB 或深度空间中，或使用 RGB 和深度数据的组合）感知机器人的环境，以便找到对象
2.在机器人上实现搜索行为的适当控制法的实现。你可以选择将其实现为简单的反应行为或更复杂的行为，例如，利用先前获得的环境地图
3.使用实施的控制律对（模拟的）Turtlebot 机器人进行电机控制
4.在执行对象搜索时尝试 SLAM
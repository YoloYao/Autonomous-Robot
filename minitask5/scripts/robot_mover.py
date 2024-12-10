#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
import math
import tf
from tf.transformations import quaternion_from_euler
from scripts.way_point import Waypoint
from scripts.robot_position import RobotPosition

# 运动控制类：按照指定坐标顺序遍历整个地图


class RobotMover:
    def __init__(self, topics_manager):
        # 定义一组导航目标点
        self.waypoints = [
            # 右房间
            Waypoint(-1.33, 2.86),   # point_1
            Waypoint(-1.17, 0.15),
            Waypoint(-0.98, -2.96),  # point_2
            Waypoint(0.22, -0.42),   # point_3
            # 上房间
            Waypoint(0.71, -3.43),   # point_4
            Waypoint(1.75, -3.76),
            Waypoint(3.07, -3.45),   # point_5
            Waypoint(3.43, -0.32),   # point_6
            # 左房间
            Waypoint(4.08, 0.80),    # point_9
            Waypoint(5.70, 2.74),    # point_7
            Waypoint(5.18, -0.14),
            Waypoint(5.45, -3.07),   # point_8
            Waypoint(4.08, 0.80),    # point_9
            # 下房间
            Waypoint(0.09, 0.92),    # point_10
            Waypoint(1.48, 4.05),    # point_11
            Waypoint(1.45, 2.50),
            Waypoint(2.76, 2.19),    # point_12
            Waypoint(3.96, 4.07),    # point_13
        ]
        self.topics_manager = topics_manager
        # 当前目标点的下标
        self.current_goal_index = 0
        self.rate = rospy.Rate(20)

    def rotate_to_goal(self, waypoint):
        # 计算目标点的朝向角度
        yaw = math.atan2(waypoint.y - self.topics_manager.pose.y,
                         waypoint.x - self.topics_manager.pose.x)
        twist = Twist()
        while abs(yaw - self.topics_manager.pose.theta) > 0.1:  # 允许一定的角度误差
            twist.angular.z = 0.8 * (yaw - self.topics_manager.pose.theta)
            self.topics_manager.cmd_pub.publish(twist)
            rospy.sleep(0.1)
        twist.angular.z = 0
        self.topics_manager.cmd_pub.publish(twist)

    def send_goal(self, waypoint):
        # 向move_base发送目标
        rospy.loginfo(f"Sending goal -> X: {waypoint.x}, Y: {waypoint.y}")
        self.rotate_to_goal(waypoint)

        # 设置导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.x
        goal.target_pose.pose.position.y = waypoint.y

        # 计算目标点的朝向角度
        yaw = math.atan2(waypoint.y - self.topics_manager.pose.y,
                         waypoint.x - self.topics_manager.pose.x)

        # 转换为四元数
        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        self.topics_manager.move_client.send_goal(
            goal)
        # 等待结果并处理失败逻辑
        retries = 0
        max_retries = 3  # 最大重试次数

        while retries < max_retries and not self.topics_manager.stop_move_base:
            success = self.topics_manager.move_client.wait_for_result(
                rospy.Duration(15.0))

            if not success:
                rospy.logwarn(
                    "Action did not complete within the timeout period.")
            else:
                state = self.topics_manager.move_client.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal reached successfully!")
                    return True
                else:
                    rospy.logwarn(f"Goal failed with state: {state}")

            retries += 1
            rospy.logwarn(f"Retrying... ({retries}/{max_retries})")
            self.topics_manager.move_client.send_goal(
                goal)

        # 如果重试超过最大次数，跳过当前目标点
        rospy.logwarn("Maximum retries reached. Skipping this goal.")
        self.topics_manager.move_client.cancel_goal()
        return False

    def move(self):
        if self.current_goal_index >= len(self.waypoints):
            return False
        self.send_goal(self.waypoints[self.current_goal_index])
        return True

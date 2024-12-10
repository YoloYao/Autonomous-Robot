#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from dynamic_reconfigure.client import Client


class SinglePointMover:
    def __init__(self):
        rospy.init_node('single_point_mover')

        # 初始化 action 客户端来发送目标
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.config_client = Client("move_base/DWAPlannerROS", timeout=5)
        # 设置目标点 (x, y, theta)
        self.target_x = -2.0
        self.target_y = 3.0
        self.target_theta = 0.0
        # 设置要调整的参数
        params = {
            "yaw_goal_tolerance": 0.4,  # 增大角度容忍度，减少旋转不稳定性
            "xy_goal_tolerance": 0.3,   # 增大距离容忍度
            "max_vel_x": 3.0,          # 最大线速度
            "max_vel_theta": 0.5,      # 最大角速度
            "min_vel_theta": 0.1       # 最小角速度
        }
        self.config_client.update_configuration(params)
        rospy.loginfo("Parameters updated successfully.")
        self.waypoints = [
            (-6.3, 0.6, 0.0),
            (-6.1, 1.6, 0.0),
            (-4.6, 3.0, 0.0),
            (-3.9, 0.5, 0.0),
            (-2.2, 0.4, 0.0),  #
            (-0.4, 0.4, 0.0),
            (0.8, 0.4, 0.0),
            (1.7, 0.3, 0.0),
            (3.4, 0.7, 0.0),
            (3.1, 2.7, 0.0),
            (3.0, 4.6, 0.0),
            (1.2, 4.6, 0.0),
            (0.8, 3.6, 0.0),
            (0.9, 1.9, 0.0)
        ]

    def send_goal(self, x, y, theta):
        # 创建一个 MoveBaseGoal 消息
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # 设置目标坐标
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = theta  # 这里使用简单的朝向设置
        goal.target_pose.pose.orientation.w = 1.0  # 角度为0的默认旋转

        # 发送目标
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

        # 等待结果并打印状态
        success = self.client.wait_for_result()
        state = self.client.get_state()
        if success and actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Reached target point (x={
                          x}, y={y}). Status: {state}")
        else:
            rospy.logwarn(f"Failed to reach target point (x={
                          x}, y={y}). Status: {state}")

    def feedback_callback(self, feedback):
        # 打印当前机器人与目标的距离差距
        rospy.loginfo(f"Current x:{feedback.base_position.pose.position.x}, y:{
                      feedback.base_position.pose.position.y}")
        rospy.loginfo(f"Aim     x:{self.target_x}, y:{self.target_y}")

    def move_to_target(self):
        for target_x, target_y, theta in self.waypoints:
            self.target_x = target_x
            self.target_y = target_y
            rospy.loginfo(f"Navigating to waypoint: ({target_x}, {target_y})")
            self.send_goal(target_x, target_y, theta)
            rospy.loginfo(f"Reached waypoint: ({target_x}, {target_y})")
            rospy.sleep(2)  # 等待 2 秒再前往下一个点


if __name__ == '__main__':
    try:
        mover = SinglePointMover()
        mover.move_to_target()
    except rospy.ROSInterruptException:
        pass

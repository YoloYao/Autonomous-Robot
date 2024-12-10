#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scripts.pose import Pose
from dynamic_reconfigure.client import Client
import tf


class TopicsManager:
    def __init__(self):
        rospy.init_node('robot_finding')

        # move publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(20)
        # 当前朝向
        self.current_yaw = None
        self.pose = Pose()
        self.move_client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.move_client.wait_for_server()
        # 当前运动状态
        self.move_status = "Search"
        # 是否停止move_base
        self.stop_move_base = False
        # 检测动作完成后开始一段间隔计时
        self.start_countdown = rospy.Time.now()
        self.max_countdown = rospy.Duration(10)
        # distance subscriber
        self.odm_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate.sleep()

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        quarternion = [orientation.x, orientation.y,
                       orientation.z, orientation.w]
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(
            quarternion)
        position = msg.pose.pose.position
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = self.current_yaw
        # 状态切换为Find并且指定停止move_base运动时，停止运动
        if self.move_status == "Find" and not self.stop_move_base:
            self.stop_move_base = True
            self.stop_move()

    def stop_move(self):
        # 取消当前目标点，停止运动
        rospy.logwarn("Cancelling current move_base goal...")
        self.move_client.cancel_goal()
        stop_cmd = Twist()
        self.cmd_pub.publish(stop_cmd)
        self.rate.sleep()

    def log_position(self):
        rospy.loginfo(
            f"Robot Position -> X: {self.pose.x}, Y: {self.pose.y}, Theta: {self.pose.theta}")

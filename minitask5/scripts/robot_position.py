#!/usr/bin/env python3
import rospy
import tf
from nav_msgs.msg import Odometry
from scripts.pose import Pose


class RobotPosition:
    def __init__(self):
        self.pose = Pose()
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = [orientation.x, orientation.y,
                      orientation.z, orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.x = position.x
        self.pose.y = position.y
        self.pose.theta = yaw

    def log_position(self):
        rospy.loginfo(
            f"Robot Position -> X: {self.pose.x}, Y: {self.pose.y}, Theta: {self.pose.theta}")

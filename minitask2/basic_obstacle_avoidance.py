#!/usr/bin/env python3

import rospy
import random
import math
import tf.transformations
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class LaserScanController:
    def __init__(self):
        # init ROS node
        rospy.init_node('laser_scan_controller', anonymous=True)
        # init publisher
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # init subscriber
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan, self.scan_callback)
        self.twist = Twist()
        # speed
        self.linear_speed = 0.2
        self.linear_speed_slow = 0.05
        self.angular_speed = 0.2
        self.angular_speed1 = -0.2
        self.angular_speed_slow = 0.05
        self.angular_speed1_slow = -0.05
        # distances
        self.safe_distance = 0.5
        self.front_dis = 100
        self.back_dis = 100
        self.left_dis = 100
        self.right_dis = 100
        self.front_ranges = None
        self.right_ranges = None
        self.rate = rospy.Rate(20)
        self.deviation_range = 45
        # counter
        self.try_max_time = 100

    def scan_callback(self, data):
        # get laser scan range data
        ranges = data.ranges
        self.front_dis = ranges[0]
        self.left_dis = ranges[90]
        self.back_dis = ranges[180]
        self.right_dis = ranges[270]
        # get the distance in front of a range of angle
        self.front_ranges = ranges[360-self.deviation_range:] + \
            ranges[:0+self.deviation_range]
        self.right_ranges = ranges[270 -
                                   self.deviation_range: 270+self.deviation_range]

    def change_direction(self):
        self.stop()
        rospy.loginfo("Robot is finding direction...")
        while min(self.front_ranges) < self.safe_distance:
            self.rotate(self.angular_speed)
        rospy.loginfo("Direction changed.")
        self.rate.sleep()
        self.move()

    def randomly_change_direction(self):
        self.stop()
        rospy.loginfo("Robot is finding direction...")
        random_speed = self.angular_speed
        # randomly turn direction
        if random.randint(0, 1) == 0:
            rospy.loginfo("Turn left")
            random_speed = self.angular_speed
        else:
            rospy.loginfo("Turn right")
            random_speed = self.angular_speed1
        while min(self.front_ranges) < self.safe_distance:
            self.rotate(random_speed)
        rospy.loginfo("Direction changed.")
        self.rate.sleep()
        self.move()

    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.move_pub.publish(self.twist)
        rospy.loginfo("Robot has stopped!")
        self.rate.sleep()

    def rotate(self, random_speed):
        self.twist.linear.x = 0.0
        self.twist.angular.z = random_speed
        self.move_pub.publish(self.twist)

    def move(self):
        self.twist.linear.x = self.linear_speed
        self.twist.angular.z = 0.0
        self.move_pub.publish(self.twist)
        rospy.loginfo("Robot is moving...")

    def obstacle_avoidance(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.front_ranges == None:
                continue
            if min(self.front_ranges) < self.safe_distance:
                self.change_direction()
            else:
                self.move()


if __name__ == '__main__':
    try:
        laser_scan_controller = LaserScanController()
        laser_scan_controller.obstacle_avoidance()
    except rospy.ROSInterruptException:
        pass

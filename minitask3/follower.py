#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class GreenObjectFollower:
    def __init__(self):
        rospy.init_node('green_object_follower', anonymous=True)
        # 订阅话题
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        # 发布移动控制命令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bridge = CvBridge()

        '''param adjusted'''
        self.stop_distance = 0.5  # 避障距离
        self.stop_area = 1200000    # 停止的面积
        self.aim_colour = "green"  # 目标颜色
        '''param adjusted'''

        self.object_detected = False
        self.target_offset = 0.0  # 目标与图像中心的偏差
        self.img_size_percent = 8
        self.mask = None
        self.centroid = 0.0
        self.area = 0.0

        self.twist = Twist()
        self.adjust_right = Twist()
        self.adjust_right.angular.z = -0.2
        self.avoid_obstacle = Twist()
        self.avoid_obstacle.angular.z = 0.3  # 原地左转，避开障碍物
        self.avoid_obstacle.linear.x = 0.1

        self.front_dis = float('inf')
        self.front_ranges = []
        self.front_dect_ang = 45
        self.range = None
        self.previous_centroid = (None, None)  # 记录上一个时刻绿色目标的位置

        self.find_obstacle = False
        self.state = "FOLLOW_TARGET"  # 初始状态为跟随目标

    def laser_callback(self, data):
        self.range = data.ranges
        max_range = data.range_max
        self.front_dis = self.range[0] if self.range[0] < max_range else max_range
        self.front_ranges = self.range[360-self.front_dect_ang:] + \
            self.range[:self.front_dect_ang]

    def image_callback(self, data):
        try:
            # 将ROS的图像消息转换为OpenCV图像
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.mask = self.get_colour_mask(hsv, self.aim_colour)
        colour_only_img = cv2.bitwise_and(image, image, mask=self.mask)
        # 调用图像处理函数来检测绿色物体
        (h, w) = image.shape[:2]
        self.object_detected, self.target_offset, self.centroid, self.area = self.process_image(
            image)
        self.previous_centroid = self.centroid
        image_resized = cv2.resize(
            image, (int(w/self.img_size_percent), int(h/self.img_size_percent)))
        colour_resized = cv2.resize(colour_only_img, (int(
            w/self.img_size_percent), int(h/self.img_size_percent)))

        cv2.imshow("original", image_resized)
        cv2.imshow("colour_mask", colour_resized)
        cv2.waitKey(3)

    def process_image(self, cv_image):
        # 寻找绿色物体的轮廓
        contours, _ = cv2.findContours(
            self.mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 找到最大的轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            M = cv2.moments(largest_contour)

            if M["m00"] > 0:
                # 计算质心
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                height, width, _ = cv_image.shape
                target_offset = cx - width // 2
                return True, target_offset, (cx, cy), area

        return False, 0, (None, None), None

    def Obstacle_detect(self):
        # 检测前方是否有障碍物
        if self.front_ranges:
            front_distance = min(self.front_ranges)  # 前方检测范围
            self.find_obstacle = front_distance < self.stop_distance
            return self.find_obstacle
        self.find_obstacle = False
        return False

    def Avoid_obstacle(self):
        rospy.loginfo("Avoiding obstacle")
        self.cmd_vel_pub.publish(self.avoid_obstacle)
        rospy.sleep(1.5)  # 等待一秒进行避障
        if not self.Obstacle_detect():
            self.cmd_vel_pub.publish(self.twist)

    def follow_green_object(self):
        # 等待激光雷达数据的更新
        rospy.loginfo("Waiting for LaserScan data...")
        while not rospy.is_shutdown() and self.range is None:
            rospy.sleep(0.1)  # 等待0.1秒以确保激光雷达数据已经更新

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 状态机控制
            if self.state == "FOLLOW_TARGET":
                if self.object_detected:
                    if not self.Obstacle_detect() and self.find_obstacle == False:
                        # 如果前方没有障碍物，继续前进并调整方向
                        self.twist.linear.x = 0.2  # 设置前进速度
                        self.twist.angular.z = - \
                            float(self.target_offset) / 1000  # 根据偏差调整角速度
                        rospy.loginfo("Moving forward.")
                    elif self.area > self.stop_area:
                        self.twist.linear.x = 0.0  # 设置前进速度
                        self.twist.angular.z = 0.0  # 根据偏差调整角速度
                        print(self.area)
                        rospy.loginfo("Stop.")
                    elif self.Obstacle_detect() and self.find_obstacle != False:
                        # 如果前方有障碍物，切换到避障状态
                        self.state = "AVOID_OBSTACLE"
                        rospy.loginfo(
                            "Obstacle detected, switching to AVOID_OBSTACLE state.")
                else:
                    # 如果没有检测到绿色物体，使用上一个记录的位置
                    if self.previous_centroid != (None, None):
                        rospy.loginfo("Using previous target position...")
                        self.twist.angular.z = - \
                            float(
                                self.previous_centroid[0] - self.img_size_percent * 4) / 1000
                        self.previous_centroid = (None, None)
                    else:
                        # 如果没有记录的位置，稍微向右旋转寻找目标
                        self.twist.angular.z = -0.2
                        self.twist.linear.x = 0.0
                        rospy.loginfo("Finding target...")

            elif self.state == "AVOID_OBSTACLE":
                # 避障状态，进行避障动作
                self.Avoid_obstacle()
                rospy.loginfo("Avoiding obstacle.")

                # 如果障碍物已经消失，返回跟随目标状态
                if self.front_dis > self.stop_distance:
                    self.state = "FOLLOW_TARGET"
                    rospy.loginfo(
                        "Obstacle cleared, switching to FOLLOW_TARGET state.")
                else:
                    # 继续避障
                    rospy.loginfo("Still avoiding obstacle.")

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

    def get_colour_mask(self, hsv, colour):
        if colour == "green":
            # the green range
            lightest = np.array([35, 70, 50])
            darkest = np.array([85, 255, 255])
            return cv2.inRange(hsv, lightest, darkest)
        elif colour == "red":
            # the red range
            lightest = np.array([0, 70, 50])
            darkest = np.array([10, 255, 255])
            lightest1 = np.array([170, 70, 50])
            darkest1 = np.array([180, 255, 255])
            mask_1 = cv2.inRange(hsv, lightest, darkest)
            mask_2 = cv2.inRange(hsv, lightest1, darkest1)
            return cv2.bitwise_or(mask_1, mask_2)
        elif colour == "yellow":
            # the yellow range
            lightest = np.array([20, 100, 100])
            darkest = np.array([30, 255, 255])
            return cv2.inRange(hsv, lightest, darkest)
        else:
            # the blue range
            lightest = np.array([100, 150, 50])
            darkest = np.array([140, 255, 255])
            return cv2.inRange(hsv, lightest, darkest)


if __name__ == '__main__':
    try:
        follower = GreenObjectFollower()
        follower.follow_green_object()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

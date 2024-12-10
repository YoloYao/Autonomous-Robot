#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# 识别控制类：目标物体的识别，避障，避开蓝色地板


class RobotFinder:
    def __init__(self):
        rospy.init_node('green_object_follower', anonymous=True)

        # 订阅话题
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 发布移动控制命令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.bridge = CvBridge()

        '''param adjusted'''
        self.stop_distance = 0.5  # 避障距离
        self.stop_green_area = 1200000    # 停止的面积
        self.stop_red_area = 180000     # red stop area
        self.aim_colour = "red"  # 目标颜色
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
        self.green_count = 0
        self.red_count = 0
        self.green_objects = []
        self.red_objects = []

    def image_callback(self, msg):
        # 把ROS图像转换为OpenCV图像
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # 把OpenCV图像转换为HSV图像
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            self.mask = self.get_colour_mask(hsv, self.aim_colour)

            # 过滤生成仅显示目标颜色的画面
            colour_only_img = cv2.bitwise_and(image, image, mask=self.mask)
            # 获取图像信息
            self.object_detected, self.target_offset, self.centroid, self.area = self.process_image(
                image, self.mask)
            # 设置相机画面大小
            (h, w) = image.shape[:2]
            image_resized = cv2.resize(image,
                                       (int(w/self.img_size_percent), int(h/self.img_size_percent)))
            colour_resized = cv2.resize(colour_only_img,
                                        (int(w/self.img_size_percent), int(h/self.img_size_percent)))
            # 显示画面
            self.image = image_resized
            self.color_image = colour_resized
            cv2.imshow("original", image_resized)
            cv2.imshow("colour_mask", colour_resized)
            cv2.waitKey(10)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def get_colour_mask(self, hsv, colour):
        if colour == "green":
            lightest = np.array([48, 165, 10])
            darkest = np.array([80, 255, 255])
            return cv2.inRange(hsv, lightest, darkest)
        elif colour == "red":
            lightest = np.array([0, 255, 10])
            darkest = np.array([5, 255, 150])
            mask_1 = cv2.inRange(hsv, lightest, darkest)
            return mask_1
        else:
            lightest = np.array([110, 160, 150])
            darkest = np.array([130, 255, 255])
            return cv2.inRange(hsv, lightest, darkest)

    def process_image(self, cv_image, mask):
        # 寻找物体的轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

        rospy.loginfo("No target detected")
        return False, 0, (None, None), None

    def move_to_target(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            print(self.area)
            rate.sleep()


if __name__ == '__main__':
    try:
        follower = RobotFinder()
        follower.move_to_target()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid

# 识别控制类：目标物体的识别，避障，避开蓝色地板


class Target:
    def __init__(self, id, color):
        # target ID
        self.ID = id
        # target color
        self.color = color
        # target centroid coord


class RobotFinder:
    def __init__(self, topics_manager):
        # 订阅话题
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.image_callback)
        # self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # 发布移动控制命令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 全局对象管理
        self.topics_manager = topics_manager

        self.bridge = CvBridge()

        self.marker_pub = rospy.Publisher(
            'visualization_marker', Marker, queue_size=5)
        self.twist = Twist()

        '''param adjusted'''
        self.stop_distance = 0.5  # 避障距离
        # mode1
        self.start_green_area = 50000    # start的面积
        self.start_red_area = 5000
        self.stop_green_area = 130000    # 停止的面积
        self.stop_red_area = 13500
        # mode2
        # self.start_green_area = 90000    # start的面积
        # self.start_red_area = 17000
        # self.stop_green_area = 380000    # 停止的面积
        # self.stop_red_area = 70000
        '''param adjusted'''
        self.aim_colour_list = ['green', 'red', 'red', 'green']
        self.color_num = 0  # 用于遍历颜色list]
        self.aim_colour = self.aim_colour_list[self.color_num]  # 目标颜色
        # self.aim_colour = "green"

        self.object_detected = False
        self.target_offset = 0.0  # 目标与图像中心的偏差
        self.img_size_percent = 2
        self.mask = None
        self.centroid = 0.0
        self.area = 0.0

        # targets
        self.green_count = 0
        self.red_count = 0
        self.green_objects = []
        self.red_objects = []

        self.twist = Twist()
        self.adjust_right = Twist()
        self.adjust_right.angular.z = -0.5
        self.avoid_obstacle = Twist()
        self.avoid_obstacle.angular.z = 0.5  # 原地左转，避开障碍物
        self.avoid_obstacle.linear.x = 0.8

        self.front_dis = float('inf')
        self.front_ranges = []
        self.front_dect_ang = 45
        self.range = None
        self.previous_centroid = (None, None)  # 记录上一个时刻绿色目标的位置

        self.find_obstacle = False

        self.stop_find = False
        # Marker ID for RViz
        self.marker_id = 0

    def show_text_in_rviz(self, text):
        marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=self.marker_id,
            lifetime=rospy.Duration(0),
            pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.6, 0.6, 0.6),
            header=Header(frame_id='base_link'),
            color=ColorRGBA(0.0, 1.0, 0.0, 0.8) if self.aim_colour == "green" else ColorRGBA(
                1.0, 0.0, 0.0, 0.8),
            text=text
        )
        self.marker_pub.publish(marker)
        self.marker_id += 1

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
            # 发现前方检测物时，切换状态为Find
            if self.is_start_find():
                self.topics_manager.move_status = "Find"
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
        return False, 0, (None, None), 0.0

    def is_start_find(self):
        # 开始Find的条件：1.目标已找到；2.目标图像大于最小要求；3.当前时间距离上次检测完成超过一定时间
        start_area = self.start_green_area if self.aim_colour == "green" else self.start_red_area
        # if self.object_detected:
        # rospy.loginfo(f"detacted:{self.object_detected}, area:{self.area}, start:{self.topics_manager.start_countdown}, time:{rospy.Time.now()}")
        if self.object_detected and self.area > start_area and rospy.Time.now() - self.topics_manager.start_countdown > self.topics_manager.max_countdown:
            return True
        else:
            return False

    def move_to_target(self):
        self.aim_colour = self.aim_colour_list[self.color_num]
        if self.aim_colour == "green":
            stop_area = self.stop_green_area
        else:
            stop_area = self.stop_red_area
        rate = rospy.Rate(10)  # 10Hz
        self.stop_find = True
        while not rospy.is_shutdown() and self.stop_find:
            # 状态机控制
            if self.object_detected:
                # 如果前方没有障碍物，继续前进并调整方向
                self.twist.linear.x = 0.2  # 设置前进速度
                self.twist.angular.z = - \
                    float(self.target_offset) / 1000  # 根据偏差调整角速度
                if self.area > stop_area:
                    object = None
                    self.twist.linear.x = 0.0  # 设置前进速度
                    self.twist.angular.z = 0.0  # 根据偏差调整角速度
                    # rospy.loginfo(f"Stop area of {self.aim_colour}:{self.area}")
                    if self.aim_colour == "green":
                        self.green_count += 1
                        object = Target(self.green_count+1, self.aim_colour)
                        self.green_objects.append(object)
                    elif self.aim_colour == "red":
                        self.red_count += 1
                        object = Target(self.red_count+1, self.aim_colour)
                        self.red_objects.append(object)
                    local_count = self.green_count if self.aim_colour == "green" else self.red_count
                    mark_word = self.aim_colour + " Box " + str(local_count)
                    mark_word = mark_word[0].upper() + mark_word[1:]
                    rospy.loginfo(f"[Target found :{mark_word}")
                    self.show_text_in_rviz(mark_word)
                    self.change_aim_color()
                    # 设置相关参数，更改当前运动状态
                    self.stop_find = False
                    self.topics_manager.move_status = "Search"
                    self.topics_manager.stop_move_base = False
                    # 刷新当前检测动作的完成时间
                    self.topics_manager.start_countdown = rospy.Time.now()

            else:
                # 如果没有记录的位置，稍微向右旋转寻找目标
                self.twist.angular.z = -0.3
                self.twist.linear.x = 0.0
                rospy.loginfo("Finding target...")

            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

    def change_aim_color(self):
        if self.color_num < len(self.aim_colour_list) - 1:
            self.color_num += 1
        else:
            self.color_num = 0
        self.aim_colour = self.aim_colour_list[self.color_num]
        #
        # if self.aim_colour == "green":
        #     self.aim_colour = "red"
        # else:
        #     self.aim_colour = "green"
        #


if __name__ == '__main__':
    try:
        follower = RobotFinder()
        follower.move_to_target()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

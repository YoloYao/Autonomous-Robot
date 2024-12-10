#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
import random
from math import pi
import math


class Pose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class FSM():
    def __init__(self):
        rospy.init_node('minitask2', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.r = rospy.Rate(30)
        # 初始化机器人的位置为(0, 0, 0)
        self.pose = Pose(0, 0, 0)
        self.is_Init_pose = False
        self.range = []
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 定义状态
        self.state = "RANDOM_MOVE"

        # 控制指令初始化
        # 停止运动
        self.default = Twist()
        # 随机前进
        self.fwd = Twist()
        self.fwd.linear.x = 0.1
        # 沿墙前进
        self.fow = Twist()
        self.fow.linear.x = 0.1
        # 沿墙向左调整
        self.adjust_left = Twist()
        self.adjust_left.angular.z = 0.15
        self.adjust_left.linear.x = 0.05
        self.quick_adjust = Twist()
        self.quick_adjust.angular.z = 0.15
        self.quick_adjust.linear.x = 0.08
        # 沿墙向右调整
        self.adjust_right = Twist()
        self.adjust_right.angular.z = -0.15
        self.adjust_right.linear.x = 0.08
        # 躲避前方障碍
        self.avoid_obstacle = Twist()
        self.avoid_obstacle.angular.z = 0.1  # 原地左转，避开障碍物

        # 增加右侧检测距离阈值和前方障碍物距离阈值
        self.d1 = 0.0  # 检测右方的范围的d1边
        self.d2 = 0.0  # 检测右方的范围的d2边
        self.last_d1 = [0.0, 0.0, 0.0]
        self.last_d2 = []
        # 距离
        self.front_dis = float('inf')
        self.right_dis = float('inf')
        self.front_ranges = None
        self.right_ranges = None
        # 最长检测距离为3.5
        self.wall_distance_threshold = 0.4
        self.front_distance_threshold = 0.4  # 前方障碍物的停止距离
        self.min_wall_distance = 0.16
        self.min_distance = 0.05
        self.max_distance = 0.4
        # 前方检测夹角
        self.front_dect_ang = 30
        # 右侧检测夹角
        self.right_dect_ang = 45

        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def shutdown(self):
        rospy.loginfo('Stopping robot')
        self.pub.publish(Twist())
        rospy.sleep(1)

    def odom_callback(self, msg):
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.is_Init_pose = True

    def laser_callback(self, data):
        self.range = data.ranges
        max_range = data.range_max
        self.front_dis = self.range[0] if self.range[0] < max_range else max_range
        self.right_dis = self.range[270] if self.range[270] < max_range else max_range

        # 设置前方检测角度范围
        self.front_ranges = self.range[360-self.front_dect_ang:] + \
            self.range[:self.front_dect_ang]
        # 过滤掉0值
        self.front_ranges = [x for x in self.front_ranges if x > 0]
        # 设置右侧检测角度范围
        self.right_ranges = self.range[270 -
                                       self.right_dect_ang: 270+self.right_dect_ang]
        # 过滤掉0值
        self.right_ranges = [x for x in self.right_ranges if x > 0]
        # 获取右侧两个角度的距离
        self.d2 = self.range[260] if self.range[260] != float('Inf') else 0.0
        self.d1 = self.range[280] if self.range[280] != float('Inf') else 0.0

    def Obstacle_detect(self):
        # 检测前方是否有障碍物
        if self.range:
            # 前方检测范围
            front_distance = min(self.front_ranges)
            return front_distance < self.front_distance_threshold
        return False

    def Wall_detect(self):
        # 检测右侧是否有墙壁
        if self.range:
            # 检测右侧范围内的激光点，通常在270度左右 ± right_dect_ang 的范围内
            right_distances = [
                d for d in self.right_ranges if self.min_distance <= d <= self.max_distance]
            # 如果满足距离条件的点数超过一定比例，则认为有墙
            # 超过80%的点在距离范围内
            if len(right_distances) >= len(self.right_ranges) * 0.9:
                return True
        return False

    def Away_wall_detect(self):
        # 检测右侧是否有墙壁
        if self.range:
            # 检测右侧范围内的激光点，通常在270度左右 ± right_dect_ang 的范围内
            right_distances = [
                d for d in self.right_ranges if self.min_distance <= d <= self.max_distance]
            # 如果满足距离条件的点数超过一定比例，则认为有墙
            # 少于50%的点在距离范围内
            if len(right_distances) <= len(self.right_ranges) * 0.5:
                return True
        return False

    def need_turn_left(self):
        if self.d1 + 0.005 < self.d2:
            return True
        return False

    def need_turn_right(self):
        if self.d1 > self.d2 + 0.005:
            return True
        return False

    def get_right_percent(self):
        # 检测右侧是否有墙壁
        if self.range:
            # 检测右侧范围内的激光点，通常在270度左右 ± right_dect_ang 的范围内
            right_distances = [
                d for d in self.right_ranges if self.min_distance <= d <= self.max_distance]
            # 如果满足距离条件的点数超过一定比例，则认为有墙
            # 超过80%的点在距离范围内
            return len(right_distances)/len(self.right_ranges)
        return 0

    def show_right_range(self):
        # 检测右侧是否有墙壁
        if self.range:
            # 检测右侧范围内的激光点，通常在270度左右 ± right_dect_ang 的范围内
            right_distances = [
                d for d in self.right_ranges if self.min_distance <= d <= self.max_distance]
            # 如果满足距离条件的点数超过一定比例，则认为有墙
            # 超过80%的点在距离范围内
            rospy.loginfo(f"Right in area:{len(right_distances)}, 60%:{
                          len(self.right_ranges) * 0.6}")
            rospy.loginfo(
                f"{len(right_distances)/len(self.right_ranges)*100}% in area")

    def Calculate_angle_deviation(self):
        # 如果某个角度没有检测到墙壁，返回0偏差（即不做角度调整）
        if abs(self.d1 - self.d2) < 0.005:
            return 0.0
        # 两点之间的角度差，转成弧度
        delta_angle = math.radians(20)
        # 计算角度偏差
        angle_deviation = math.atan((self.d1 - self.d2) / delta_angle)
        return angle_deviation  # 正值表示机器人需向左调整，负值表示需向右调整

    def Follow_wall(self):
        rospy.loginfo("Following the wall")
        # 检测前方障碍
        if self.Obstacle_detect():
            self.Stop()
            rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
            self.state = "AVOID_OBSTACLE"
            return

        while "FOLLOW_WALL" == self.state:
            self.show_right_range()
            if self.Obstacle_detect():
                self.Stop()
                rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
                self.state = "AVOID_OBSTACLE"
                return
            # 检测到离开墙面
            if self.Away_wall_detect():
                self.Stop()
                # 1.向右调整继续贴墙
                # self.state = "ADJUST_TO_WALL"
                # 2.开始随机行驶
                self.state = "RANDOM_MOVE"
                return
            # 顺墙走的几种运动
            # 距离墙面太近，快速调整向左
            if self.right_dis <= self.min_wall_distance and (self.need_turn_left() or self.Wall_detect()):
                self.Adjust_away_wall()
            elif self.need_turn_left():
                self.Adjust_left()
            elif self.need_turn_right():
                self.Adjust_right()
            else:
                # 正常沿墙前进
                self.pub.publish(self.fow)
                self.r.sleep()
                rospy.loginfo("Following wall")

    # 原地调整向右转至墙面
    def Adjust_to_wall(self):
        rospy.loginfo("Adjusting to find wall")
        max_times = 100
        try_times = 0
        while not self.Wall_detect() and try_times < max_times:
            if self.Obstacle_detect():
                self.Stop()
                rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
                self.state = "AVOID_OBSTACLE"
                return
            # 向右调整
            rospy.loginfo("Adjusting to wall.")
            adjust_info = Twist()
            adjust_info.linear.x = 0
            adjust_info.angular.z = -0.1
            self.pub.publish(adjust_info)
            self.r.sleep()
            try_times += 1

        self.Stop()
        rospy.loginfo("Adjusted to wall finished.")
        self.state = "FOLLOW_WALL"

    # 距离右墙太近时，需要快速调整离开墙面
    def Adjust_away_wall(self):
        rospy.loginfo("Adjusting away the wall")
        max_times = 60
        try_times = 0
        while self.right_dis <= self.min_wall_distance and try_times < max_times:
            if self.Obstacle_detect():
                self.Stop()
                rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
                self.state = "AVOID_OBSTACLE"
                return
            # 向左调整
            rospy.loginfo("Adjusting away wall.")
            self.pub.publish(self.quick_adjust)
            self.r.sleep()
            try_times += 1

        self.Stop()
        rospy.loginfo("Adjusted away wall finished.")
        self.state = "FOLLOW_WALL"

    def Adjust_left(self):
        rospy.loginfo("Adjusting left to avoid close wall")
        self.pub.publish(self.adjust_left)
        self.r.sleep()
        # 检测是否恢复到理想距离
        if abs(self.d1 - self.d2) < 0.005:
            self.state = "FOLLOW_WALL"

    def Adjust_right(self):
        rospy.loginfo("Adjusting right to get closer to wall")
        self.pub.publish(self.adjust_right)
        self.r.sleep()
        # 检测是否恢复到理想距离
        if abs(self.d1 - self.d2) < 0.005:
            self.state = "FOLLOW_WALL"

    def Stop(self):
        self.pub.publish(self.default)
        rospy.loginfo("Robot has stopped!")
        self.r.sleep()

    def Rotate(self, angle):
        rospy.loginfo("Random rotation start")
        spin_accum = 0
        ori_theta = self.pose.theta
        rotation_speed = Twist()
        rotation_speed.angular.z = 0.3 if angle > 0 else -0.3

        while abs(spin_accum) < abs(angle):
            diff = self.pose.theta - ori_theta
            if angle > 0 and diff < 0:
                diff += 2 * pi
            elif angle < 0 and diff > 0:
                diff -= 2 * pi

            spin_accum += diff
            ori_theta = self.pose.theta
            self.pub.publish(rotation_speed)
            self.r.sleep()

        self.pub.publish(self.default)
        rospy.loginfo("Random rotation complete")

    def Random_move(self):
        rospy.loginfo("Random move initiated")
        self.state = "RANDOM_MOVE"
        ori_x = self.pose.x
        ori_y = self.pose.y
        dis_accum = 0
        while dis_accum < 1.0:
            if self.Obstacle_detect():
                rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
                self.state = "AVOID_OBSTACLE"
                # self.pub.publish(self.default)
                self.Stop()
                return

            # 如果在随机移动过程中检测到墙壁，立即切换到沿墙状态
            if self.Wall_detect():
                rospy.loginfo("Wall detected, switching to FOLLOW_WALL")
                self.state = "FOLLOW_WALL"
                # self.pub.publish(self.default)
                self.Stop()
                return
            rospy.loginfo("Random moving.")
            self.pub.publish(self.fwd)
            self.r.sleep()
            dis_accum = math.sqrt((self.pose.x - ori_x) **
                                  2 + (self.pose.y - ori_y)**2)

        self.pub.publish(self.default)
        rospy.sleep(1)
        # 随机旋转
        random_angle = random.uniform(-pi/2, pi/2)
        self.Rotate(random_angle)
        self.state = "RANDOM_MOVE"

    def Avoid_obstacle(self):
        rospy.loginfo("Avoiding obstacle")
        self.pub.publish(self.avoid_obstacle)
        self.r.sleep()
        if not self.Obstacle_detect() and 0.0 < abs(self.range[275] - self.range[265]) < 0.005:
            # 根据是否检测到墙来决定回到哪个状态
            self.Stop()
            self.state = "FOLLOW_WALL" if self.Wall_detect() else "RANDOM_MOVE"

    def Driving(self):
        # 等待激光雷达数据的更新
        rospy.loginfo("Waiting for LaserScan data...")
        while not rospy.is_shutdown() and self.range != []:
            self.r.sleep()
        while self.is_Init_pose == False:
            self.r.sleep()
        # 在启动时优先检查右侧是否有墙壁
        if self.Wall_detect():
            rospy.loginfo("Initial wall detected, switching to FOLLOW_WALL")
            self.state = "FOLLOW_WALL"

        # 在启动时优先检查右侧是否有墙壁
        if self.Obstacle_detect():
            rospy.loginfo(
                "Initial obstacle detected, switching to AVOID_OBSTACLE")
            self.state = "AVOID_OBSTACLE"

        while not rospy.is_shutdown():
            if self.Obstacle_detect() and "AVOID_OBSTACLE" != self.state:
                rospy.loginfo("Obstacle detected, switching to AVOID_OBSTACLE")
                self.state = "AVOID_OBSTACLE"

            if "FOLLOW_WALL" == self.state:
                self.Follow_wall()
            elif "ADJUST_TO_WALL" == self.state:
                self.Adjust_to_wall()
            elif "RANDOM_MOVE" == self.state:
                self.Random_move()
            elif "AVOID_OBSTACLE" == self.state:
                self.Avoid_obstacle()


if __name__ == '__main__':
    try:
        # 创建Driving实例并传入初始位姿
        run = FSM()
        # 开始运行driving方法
        run.Driving()
    except rospy.ROSInterruptException:
        pass

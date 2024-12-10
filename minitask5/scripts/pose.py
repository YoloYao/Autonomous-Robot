#!/usr/bin/env python3
import math


class Pose:
    # 初始化机器人位置
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    # 计算两个点之间的距离
    @staticmethod
    def calculate_distance(p1, p2):
        return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)

    # 将角度规范化到[-pi, pi]范围内
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

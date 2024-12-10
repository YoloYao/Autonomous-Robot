#!/usr/bin/env python3

import traceback
import rospy
import sys
import os
cur_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(1, cur_path)
from scripts.topics_manager import TopicsManager
from scripts.robot_mover import RobotMover
from scripts.robot_finder import RobotFinder

class RobotController:
    def init(self):
        # 全局参数管理对象
        self.topics_manager = TopicsManager()
        # 运动控制
        self.robotMover = RobotMover(self.topics_manager)
        # 识别控制
        self.robotFinder = RobotFinder(self.topics_manager)
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        rospy.loginfo("Shutting down... Cleaning up!")
        self.topics_manager.stop_move()

    def run(self):
        rospy.loginfo("[Start find mission]")
        while not rospy.is_shutdown():
            if "Search" == self.topics_manager.move_status:
                rospy.loginfo("State1: Moving")
                result = self.robotMover.move()
                if not result:
                    break
                self.robotMover.current_goal_index += 1
            elif "Find" == self.topics_manager.move_status:
                rospy.loginfo("State2: Finding")
                self.robotFinder.move_to_target()
                rospy.sleep(1.0)
                self.robotMover.current_goal_index += 1
        rospy.loginfo("Finish searching each of points.")


if __name__ == "__main__":
    try:
        robotController = RobotController()
        robotController.init()
        robotController.run()
    except Exception as e:
        rospy.loginfo(f"ERROR:{e}")
        traceback.print_exc()

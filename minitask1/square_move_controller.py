#!/usr/bin/env python3

import rospy
import math
import tf.transformations
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow, pi


class SquareMoveController:
    def init(self):
        # init node
        rospy.init_node('square_move_controller', anonymous=True)
        # init position publisher
        self.init_pose_pub = rospy.Publisher(
            '/initialpose', PoseWithCovarianceStamped, queue_size=10)
        # distance subscriber
        self.odm_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # move publisher
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # position
        self.current_pose = None
        self.start_pose = None
        self.initial_yaw = None
        self.current_yaw = 0.0
        self.target_yaw = None
        # speed
        self.rate = rospy.Rate(20)
        self.linear_speed_fast = 0.15
        self.linear_speed_mid = 0.10
        self.linear_speed_slow = 0.05
        self.angular_speed = 2.0
        # distance
        self.side_length = 1.0
        self.side_length1 = 0.95
        self.side_length2 = 0.85
        self.side_length3 = 0.1
        # count
        self.square_aim_times = 1
        self.edges_completed = 0.0

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.current_yaw = self.get_direction(self.current_pose.orientation)

    def straight_move(self):
        # set aim yaw
        self.set_initial_yaw()
        move_info = Twist()
        move_info.linear.x = self.linear_speed_mid
        move_info.angular.z = 0.0
        # rospy.loginfo("Position: x=%f, y=%f, z=%f", self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)
        self.start_pose = self.current_pose
        start_x = self.start_pose.position.x
        start_y = self.start_pose.position.y

        distance_moved = 0.0

        while distance_moved < self.side_length:
            # calculate current deviation angle
            yaw_error = self.normalize_angle(
                self.initial_yaw - self.current_yaw)
            # change speed by deviation angle
            move_info.angular.z = yaw_error
            # change speed
            if distance_moved > self.side_length1:
                move_info.linear.x = self.linear_speed_slow
            elif distance_moved > self.side_length2:
                move_info.linear.x = self.linear_speed_mid
            elif distance_moved > self.side_length3:
                move_info.linear.x = self.linear_speed_fast
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            distance_moved = sqrt(
                pow((current_x - start_x), 2) + pow((current_y - start_y), 2))
            self.move_pub.publish(move_info)
            self.rate.sleep()

        move_info.linear.x = 0
        self.move_pub.publish(move_info)
        self.rate.sleep()

    def turn_90_degrees(self):
        self.set_initial_yaw()
        # set rotation to 90 degrees from the current direction
        self.target_yaw = self.normalize_angle(
            self.initial_yaw + math.radians(90))
        # rospy.loginfo("Aim yaw:%f   Current yaw:%f  Target yaw:%f", self.initial_yaw, self.current_yaw, self.target_yaw)
        turn_info = Twist()
        turn_info.linear.x = 0
        turn_info.angular.z = self.angular_speed

        direction_turned = 0.0

        while direction_turned < (pi / 2):
            direction_turned = self.normalize_angle(
                self.current_yaw - self.initial_yaw)
            yaw_error = self.normalize_angle(
                self.target_yaw - self.current_yaw)
            # set the angular speed
            if abs(yaw_error) > 0.01:
                turn_info.angular.z = yaw_error
            else:
                # stop rotating when the angle reaches the target
                turn_info.angular.z = 0
                break
            self.move_pub.publish(turn_info)
            self.rate.sleep()

        turn_info.angular.z = 0
        self.move_pub.publish(turn_info)
        self.rate.sleep()

    def set_initial_yaw(self):
        if self.edges_completed == 0:
            self.initial_yaw = 0
        elif self.edges_completed == 1:
            self.initial_yaw = math.pi/2
        elif self.edges_completed == 2:
            self.initial_yaw = math.pi
        elif self.edges_completed == 3:
            self.initial_yaw = -math.pi/2

    def init_direction(self):
        rospy.loginfo("Start initial direction initial...")
        self.rate.sleep()
        self.set_initial_yaw()
        target_yaw = self.initial_yaw
        move_info = Twist()
        move_info.linear.x = 0.0
        move_info.angular.z = 0.0
        while self.current_yaw == None:
            rospy.loginfo("Waiting for odom update until valid yaw.")
        while abs(self.normalize_angle(self.current_yaw - target_yaw)) > 0.01:
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            move_info.angular.z = yaw_error
            self.move_pub.publish(move_info)
            self.rate.sleep()
        move_info.linear.x = 0
        self.move_pub.publish(move_info)
        self.rate.sleep()
        rospy.loginfo("Direction initial succeeded!")

    @staticmethod
    def normalize_angle(angle):
        # limit the angle to [-pi,pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_direction(self, orientation):
        orientation_list = [orientation.x,
                            orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        return yaw

    def run(self):
        self.rate.sleep()
        # square run times
        square_count = 0
        while self.current_pose == None:
            rospy.loginfo('Searching...')
        while square_count < self.square_aim_times:
            while not rospy.is_shutdown() and self.edges_completed < 4:
                # straight move
                rospy.loginfo('Start straight path %d.', self.edges_completed + 1)
                self.straight_move()
                rospy.loginfo('Completed straight path %d.',
                            self.edges_completed + 1)

                # turn 90 degrees
                rospy.loginfo('Start 90-degree rotation %d.',
                            self.edges_completed + 1)
                self.turn_90_degrees()
                rospy.loginfo('Completed 90-degree rotation %d.',
                            self.edges_completed + 1)

                self.edges_completed += 1
            self.edges_completed = 0
            square_count += 1

        rospy.loginfo("Completed square move!")
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = SquareMoveController()
        controller.init()
        # controller.init_direction()
        controller.run()
    except rospy.ROSInterruptException:
        pass
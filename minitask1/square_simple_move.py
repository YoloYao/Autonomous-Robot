#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def square_mover():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('squareMove', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    rate.sleep()
    side_length = 1.1
    side_speed = 0.25
    side_time = side_length / side_speed
    angular_speed = 0.5
    angular_time = 3.2
    msg = Twist()
    for num in range(0, 1):
        start_time = rospy.Time.now().to_sec()  # startTime
        rospy.loginfo(start_time)
        while (rospy.Time.now().to_sec() - start_time) < side_time:
            rospy.loginfo(rospy.Time.now().to_sec() - start_time)
            msg.linear.x = side_speed
            msg.angular.z = 0
            pub.publish(msg)
        stop_move(msg, pub, rate)
        start_time = rospy.Time.now().to_sec()  # startTime
        rospy.loginfo(start_time)
        while (rospy.Time.now().to_sec() - start_time) < angular_time:
            rospy.loginfo(rospy.Time.now().to_sec() - start_time)
            msg.linear.x = 0
            msg.angular.z = angular_speed
            pub.publish(msg)
        stop_move(msg, pub, rate)


def stop_move(msg, pub, rate):
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    rate.sleep()
    rospy.loginfo('stop')


if __name__ == '__main__':
    try:
        square_mover()
    except rospy.ROSInterruptException:
        pass

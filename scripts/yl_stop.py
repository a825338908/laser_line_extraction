#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('stop')
    loop_rate = rospy.Rate(1)  # 10hz
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub_cmd_vel.publish(twist)
        loop_rate.sleep()
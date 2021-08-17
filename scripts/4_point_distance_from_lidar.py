#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.msg
import redis 

red = redis.Redis(host='localhost', port=6379)
scann = LaserScan()

red.hset("lidar_4_points", "front", 0)
red.hset("lidar_4_points", "rear", 0)
red.hset("lidar_4_points", "left", 0)
red.hset("lidar_4_points", "right", 0)




def callback(msg):
    #print(len(msg.ranges)) len is 2019 from 0-360
    #print("length: " + str(len(msg.ranges)))
    front_idx = 720 * 180/360
    front_measure = float(msg.ranges[front_idx])
    red.hset("lidar_4_points", "front", front_measure)
    print("front: " + str(front_measure))

    rear_idx = 720 * 0/360
    rear_measure = float(msg.ranges[rear_idx])
    red.hset("lidar_4_points", "rear", rear_measure)
    print("rear: " + str(rear_measure))

    left_idx =  720 * 270/360
    left_measure = msg.ranges[left_idx]
    red.hset("lidar_4_points", "left", left_measure)
    print("left: " + str(left_measure))


    right_idx = 720 * 90/360
    right_measure = msg.ranges[right_idx]
    red.hset("lidar_4_points", " right", right_measure)
    print("right: " + str(right_measure))





def listener():
    rospy.init_node('revised_scan', anonymous=True)
    sub = rospy.Subscriber('/scan_front', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
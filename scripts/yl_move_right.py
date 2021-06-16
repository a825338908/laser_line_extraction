#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math
from laser_line_extraction.msg import LineSegment, LineSegmentList


class AutoPaint():
    def __init__(self):

        self.walls = None
        self.is_walls_available = False
        loop_rate = rospy.Rate(10)  # 10Hz in gazebo

        # self.current_stage = 'adjust_distance'
        # self.current_stage = 'adjust_orientation'
        # self.current_stage = 'move_right'
        # self.current_stage = 'rot90'
        self.current_stage = 'move_right'
        self.rot90_t0 = None

        '''
        debug
        adjust_orientation
        adjust_distance        
        '''
        self.sub_lines = rospy.Subscriber('/line_segments', LineSegmentList, self.cbGetLines, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.stage_num = 0
        rospy.on_shutdown(self.fnShutDown)
        while not rospy.is_shutdown():
            if self.is_walls_available is True:
                self.process()
            loop_rate.sleep()
        print('88')

    def process(self):
        if len(self.walls) == 0:
            return
        angle, distance = self.walls[0]
        if self.current_stage == 'init':
            print(self.walls)
        elif self.current_stage == 'adjust_orientation':
            print('adjust orientation...')
            print(angle)
            if angle < -1 or angle > 1:
                self.fnTurn(-angle * math.pi / 180)
            else:
                self.fnStop()
                self.stage_num += 1
        elif self.current_stage == 'adjust_distance':
            print('adjust distance...', distance)
            target_distance = 100  # cm
            if abs(distance - target_distance) > 10:
                self.fnGoStraight(y=(distance - target_distance) / 100)
            else:
                self.fnStop()
                self.stage_num += 1
        elif self.current_stage == 'move_right':
            target_distance = 130  # cm
            print(distance - target_distance)

            if distance - target_distance> 10:
                self.fnGoStraight(x=-0.05, fixed_speed=True)
            elif 10 > distance - target_distance> 0:
                self.fnGoStraight(x=-1*(distance - target_distance), kp=0.01)
            elif 0 > distance - target_distance> -10:
                self.fnGoStraight(x=-(distance - target_distance), kp=0.01)
        elif self.current_stage == 'rot90':
            if self.rot90_t0 is None:
                self.rot90_t0 = rospy.get_time()
            print(rospy.get_time() - self.rot90_t0)
            if (rospy.get_time() - self.rot90_t0) > 10:
                self.fnStop()
                self.current_stage == 'adjust_orientation'
                self.stage_num = 1
            else:
                self.fnTurn(-math.pi * 0.1 / 2)

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def cbGetLines(self, lines_msg):
        angles = []
        distances = []
        if len(lines_msg.line_segments) > 0:
            for line in lines_msg.line_segments:
                angle_deg = line.angle * 180 / math.pi
                angle_deg = round(angle_deg)
                distance_cm = round(100 * line.radius)

                found = False
                merge_cm = 5
                merge_deg = 5
                for i in range(len(angles)):
                    if angles[i] - merge_deg <= abs(angle_deg) <= angles[i] + merge_deg:
                        if distances[i] - merge_cm <= distance_cm <= distances[i] + merge_cm:
                            found = True
                            break
                if not found:
                    angles.append(angle_deg)
                    distances.append(distance_cm)

        walls = zip(angles, distances)
        if self.current_stage == 'adjust_orientation':
            walls = sorted(walls, key=lambda x: abs(x[0]), reverse=False)
        elif self.current_stage == 'adjust_distance':
            walls = sorted(walls, key=lambda x: abs(x[1]), reverse=False)
        elif self.current_stage == 'move_right':
            walls = sorted(walls, key=lambda x: abs(x[0] + 90), reverse=False)
        else:
            walls = sorted(walls, key=lambda x: abs(x[0]), reverse=False)
        self.walls = walls
        if not self.is_walls_available:
            self.is_walls_available = True

    def fnStop(self):
        self.fnGoStraight(0, 0, 0)

    def fnTurn(self, theta):
        Kp = 0.8

        angular_z = Kp * theta
        angular_z = self.clamp(angular_z, -math.pi / 10, math.pi / 10)

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fnGoStraight(self, x=0, y=0, z=0, limit=0.1, fixed_speed=False, kp=0.8):
        if not fixed_speed:
            Kp = kp
            x = Kp * x
            y = Kp * y
            z = Kp * z
            x = self.clamp(x, -limit, limit)
            y = self.clamp(y, -limit, limit)
            z = self.clamp(z, -limit, limit)
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist)

    def fnGet2DRobotPose(self, robot_odom_msg):
        quaternion = (robot_odom_msg.pose.pose.orientation.x, robot_odom_msg.pose.pose.orientation.y,
                      robot_odom_msg.pose.pose.orientation.z, robot_odom_msg.pose.pose.orientation.w)
        theta = tf.transformations.euler_from_quaternion(quaternion)[2]

        if theta < 0:
            theta = theta + np.pi * 2
        if theta > np.pi * 2:
            theta = theta - np.pi * 2

        pos_x = robot_odom_msg.pose.pose.position.x
        pos_y = robot_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    def fnCalcDistPoints(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.fnStop()

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('auto_paint_vision')
    node = AutoPaint()
    node.main()

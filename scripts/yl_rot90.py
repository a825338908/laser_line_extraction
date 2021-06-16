#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math
from laser_line_extraction.msg import LineSegment, LineSegmentList


class AutoPaint():
    def __init__(self):
        self.walls = None
        self.is_walls_available = False
        loop_rate = rospy.Rate(10)  # 10Hz in gazebo
        self.current_stage = 'rot90'
        self.sub_lines = rospy.Subscriber('/line_segments', LineSegmentList, self.cbGetLines, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.fnShutDown)
        self.rot90_t0 = None
        rospy.on_shutdown(self.fnShutDown)
        while not rospy.is_shutdown():
            if self.is_walls_available is True:
                self.process()
            loop_rate.sleep()
        print('88')

    def process(self):
        if self.current_stage == 'rot90':
            if self.rot90_t0 is None:
                self.rot90_t0 = rospy.get_time()
            elif (rospy.get_time() - self.rot90_t0) > 10.0: #seconds
                self.fnStop()
                rospy.signal_shutdown("rot90 job done")
            else:
                print(rospy.get_time() - self.rot90_t0)
                self.fnTurn(math.pi*0.5/2)

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)


    def fnStop(self):
        self.fnGoStraight(0, 0, 0)

    def fnTurn(self, theta):
        Kp = 0.8

        angular_z = Kp * theta
        angular_z = self.clamp(angular_z, -math.pi , math.pi )

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fnGoStraight(self, x=0, y=0, z=0, limit=0.1, fixed_speed=False):
        if not fixed_speed:
            Kp = 0.8
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

    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")
        self.fnStop()


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


    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('auto_paint_vision_cw_90')
    node = AutoPaint()
    node.main()

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from laser_line_extraction.msg import LineSegment, LineSegmentList
import numpy as np


class AutoPaint():
    def __init__(self):
        self.min_spacing_mm = 800
        self.tolerance_distance = 50  # mm
        self.walls = None
        self.is_walls_available = False
        loop_rate = rospy.Rate(2)  # 1Hz in gazebo
        self.current_stage = 'debug'  # read_flow
        self.sub_lines = rospy.Subscriber('/line_segments', LineSegmentList, self.cbGetLines, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.step_t0 = None
        self.step_d0 = None
        self.step_num = None
        self.steps = None
        self.loops = dict()
        #loop: name: {step_start_num, step_end_num}

        rospy.on_shutdown(self.fnShutDown)
        while not rospy.is_shutdown():
            if self.is_walls_available is True:
                keep_working = self.process()
                if keep_working and self.step_num < len(self.steps):
                    pass
                else:
                    rospy.loginfo("Completed")
                    rospy.signal_shutdown("Completed")
                    break
            loop_rate.sleep()
        print('898')

    def register_steps(self, lines):
        steps = []
        for line in lines:
            if line.startswith('#') or line.startswith('//'):
                pass
            else:
                steps.append(line)
        self.steps = steps

    def register_loops(self):
        for i in range(len(self.steps)):
            step = self.steps[i]
            tmp = step.split(';')
            if tmp[0] == 'loop' and tmp[1] == 'start':
                loop_name = tmp[2]
                loop_step_start_num = i
                loop_step_end_num = -1
                j = i+1
                while j < len(self.steps):
                    another_step = self.steps[j]
                    another_step_tmp = another_step.split(';')
                    if another_step_tmp[0] == 'loop' and another_step_tmp[1] == 'end' and another_step_tmp[2] == loop_name:
                        loop_step_end_num = j
                        break
                    j+=1
                if loop_step_end_num < 0: #TODO error
                    rospy.logfatal("can't find loop end {}".format(loop_name))
                self.loops[loop_name] = (loop_step_start_num, loop_step_end_num)
        rospy.loginfo(self.loops)

    def get_walls(self, order_by_angle=None):
        if not self.is_walls_available:
            return None

        walls = self.walls
        if order_by_angle is not None:
            walls = sorted(walls, key=lambda x: abs(x[0] - order_by_angle),
                           reverse=False)  # TODO something strange. come back later
        print(walls)
        return walls

    def process(self):
        step_keep_running = False
        if len(self.walls) == 0:
            return True
        if self.current_stage == 'debug':
            print(self.get_walls(order_by_angle=0))
            return False
        elif self.current_stage == 'bye':
            return True
        elif self.current_stage == 'read_flow':
            file1 = open('aicode.txt', 'r')
            #file1 = open('aicode_rotate.txt', 'r')
            lines = file1.readlines()
            # todo validate lines
            self.register_steps(lines)
            self.step_num = 0
            self.step_t0 = rospy.get_time()
            self.register_loops()
            self.current_stage = 'process_flow'
        elif self.current_stage == 'process_flow':
            rospy.loginfo("current step#: {}".format(self.step_num))
            step = self.steps[self.step_num]
            print(step)
            if step.strip('') == '':
                pass #proceed to next step
            else:
                tmp = step.split(';')
                if tmp is None or len(tmp) == 0:  # todo validate more later
                    rospy.logerr('parse step error. Step#{}'.format(self.step_num))
                    return False #exit
                command = tmp[0]
                if command == 'wait':
                    ms = float(tmp[1])
                    if self.step_t0 is None:
                        self.step_t0 = rospy.get_time()
                    else:
                        remaining = round((self.step_t0 + ms/1000) - rospy.get_time(),1)
                        if remaining > 0:
                            rospy.loginfo('waiting..remaining: {}'.format(remaining))
                            step_keep_running = True
                        else:
                            try:
                                #rospy.loginfo(tmp[2])
                                print(tmp[2])
                            except:
                                pass

                elif command == 'adjust':
                    if tmp[1] == 'distance':
                        walls = self.get_walls(float(tmp[3]))
                        target_distance = float(tmp[4])
                        angle, distance = walls[0]
                        if tmp[2] == 'start':
                            target_distance = -1*target_distance
                            if self.step_d0 is None:
                                self.step_d0 = distance
                            target_distance += self.step_d0

                        if target_distance < self.min_spacing_mm:
                            target_distance = self.min_spacing_mm

                        rospy.loginfo('target distance to wall: {}'.format(target_distance))
                        rospy.loginfo('current distance to wall: {}'.format(distance))
                        if abs(distance - target_distance) > self.tolerance_distance:  # mm
                            # assuming y direction
                            if -1 < float(tmp[3]) < 1:
                                self.fnGoStraight(x=(distance - target_distance) / 100)
                            elif 89 < float(tmp[3]) < 91:
                                self.fnGoStraight(y=(distance - target_distance) / 100)
                            elif -91 < float(tmp[3]) < -89:
                                self.fnGoStraight(y=-(distance - target_distance) / 100)
                            elif -181 < float(tmp[3]) < -179:
                                self.fnGoStraight(x=-(distance - target_distance) / 100)
                            step_keep_running = True
                        else:
                            step_keep_running = False
                    elif tmp[1] == 'orientation':
                        walls = self.get_walls(float(tmp[3]))
                        angle, distance = walls[0]
                        target = float(tmp[4])
                        rospy.loginfo('current angle to wall: {}'.format(angle))
                        rospy.loginfo('target angle to wall: {}'.format(target))
                        print((angle - target))
                        if (angle - target) > 1:
                            self.fnTurn(-1* abs(angle - target) * math.pi / 180)
                            step_keep_running = True
                        elif (angle - target) <- 1:
                            self.fnTurn(abs(angle - target) * math.pi / 180)
                            step_keep_running = True
                        else:
                            step_keep_running = False
                elif command == 'loop':
                    if tmp[1] == 'start':
                        step_keep_running = False
                    elif tmp[1] == 'end':
                        step_keep_running = False
                        self.step_t0 = None
                        self.step_num, _ = self.loops[tmp[2]] #goto step
                    elif tmp[1] == 'break_if':
                        #check condition, if met, go to end
                        broken_loop = False
                        print('break_if')
                        if tmp[3] == 'orientation':
                            walls = self.get_walls(float(tmp[5]))
                            angle, distance = walls[0]
                            target = float(tmp[6])
                            rospy.loginfo('current angle to wall: {}'.format(angle))
                            if angle < target - 1 or angle > target + 1:
                                pass #do nothing  #proceed to next step
                            else:
                                # break the loop
                                broken_loop = True
                        elif tmp[3] == 'distance':
                            walls = self.get_walls(float(tmp[5]))
                            target_distance = float(tmp[6])
                            angle, distance = walls[0]
                            if tmp[4] == 'start':
                                target_distance = -1 * target_distance + distance
                            rospy.loginfo('target distance to wall: {}'.format(target_distance))
                            rospy.loginfo('current distance to wall: {}'.format(distance))
                            if distance < target_distance:  # mm
                                broken_loop = True

                        if broken_loop:
                            step_keep_running = False
                            self.step_t0 = None
                            _, self.step_num = self.loops[tmp[2]]  # goto end step
                            rospy.loginfo("loop broken")
                        else:
                            step_keep_running = False
                else:
                    rospy.logwarn("command not implemented. command: {}".format(command))
                    # process to next command
            if not step_keep_running:
                print('next step')
                self.fnStop()
                self.step_t0 = rospy.get_time()
                self.step_d0 = None
                self.step_num += 1
        return True

    def clamp(self, n, minn, maxn):
        return max(min(maxn, n), minn)

    def cbGetLines(self, lines_msg, order_by_angle=None):
        angles = []
        distances = []
        if len(lines_msg.line_segments) > 0:
            for line in lines_msg.line_segments:
                angle_deg = line.angle * 180 / math.pi
                if angle_deg > 175:
                    angle_deg = -180.0
                angle_deg = round(angle_deg)
                distance_cm = round(100 * line.radius)

                found = False
                merge_cm = 5
                merge_deg = 5
                found = False
                for i in range(len(angles)):
                    check_angle = angles[i]
                    check_distance = distances[i]
                    if abs(check_angle) - merge_deg < abs(angle_deg) < abs(check_angle) + merge_deg:
                        found = True
                        if distance_cm > check_distance:
                            found = True
                            break
                if not found:
                    angles.append(angle_deg)
                    distances.append(distance_cm)  # mm
        distances = np.array(distances) * 10
        walls = zip(angles, distances)
        self.walls = sorted(walls, key=lambda x: abs(x[1]), reverse=False)  # sort by distance by default
        if not self.is_walls_available:
            self.is_walls_available = True

    def fnStop(self):
        self.fnGoStraight(0, 0, 0)

    def fnTurn(self, theta):
        Kp = 2.0

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

    def fnGoStraight(self, x=0, y=0, z=0, limit=0.3, fixed_speed=False):
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

    def main(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('auto_paint_vision_face_wall_angle', anonymous=True)
    node = AutoPaint()
    node.main()

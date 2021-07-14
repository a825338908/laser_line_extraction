#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
from laser_line_extraction.msg import LineSegment, LineSegmentList
import numpy as np
import redis
from enum import Enum
import time
import sys

move_x_dir = False
move_y_dir = False

class SliderAction(Enum):
    STOP = -2.0
    DOWN = -1.0
    UP = 1.0
    NONE = 0.0


class SprayAction(Enum):
    ON = 1.0
    OFF = 0.0


class SliderDevice(Enum):
    MAIN = 'slider_main'


class SprayDevice(Enum):
    HIGH = 'spray_mid' #'spray_high'
    MID = 'spray_high'#spray_mid'
    LOW = 'spray_low'


def spray_device_from_key(key):
    if key == 'spray_high':
        device = SprayDevice.HIGH
    elif key == 'spray_mid':
        device = SprayDevice.MID
    elif key == 'spray_low':
        device = SprayDevice.LOW
    return device


def spray_device_from_id(id):
    if id == 0:
        device = SprayDevice.HIGH
    elif id == 1:
        device = SprayDevice.MID
    elif id == 2:
        device = SprayDevice.LOW
    return device


def init_redis(r=None, restore_default=False):
    # slider_main_height_laser
    if r is None:
        r = redis.Redis()
    #r.set('spray_main_switch', 1.0)
    #r.hset("Joy", "spray_low", 0.0)  # 0.0 or 1.0
    #r.hset("Joy", "spray_mid", 0.0)  # 0.0 or 1.0
    #r.hset("Joy", "spray_high", 0.0)  # 0.0 or 1.0
    #r.hset("Joy", "updown_but", 0.0)  # -1.0 or 0.0 or 1.0

    if restore_default:
        r.set("spray_main_switch", 0.0)
        r.hset("spray_high_range", "min", 0)
        r.hset("spray_high_range", "max", 2100)
        r.hset("spray_mid_range", "min", 0)
        r.hset("spray_mid_range", "max", 2100)
        r.hset("spray_low_range", "min", 0)
        r.hset("spray_low_range", "max", 2100)

        r.hset("slider_main_range", "min", 0)
        r.hset("slider_main_range", "max", 2100)

    # high: 1560 < x < 2630
    # mid: 890 < x < 2630
    # low: 1020 < x < 1910
    return r


class SliderPaint():

    def __init__(self, redis_conn=None, slider_name='main'):
        if redis_conn is None:
            self.r = init_redis()
        else:
            self.r = redis_conn
        self.name = slider_name
        self.default_slider_move_timeout_ms = 1000000
        self.min_pos = float(self.r.hget('slider_' + slider_name + '_range', 'min'))
        self.max_pos = float(self.r.hget('slider_' + slider_name + '_range', 'max'))
        self.is_running = False
        self.is_processing_command = False
        self.tolerance_mm = 50
        self.processing_action = None
        self.spray_devices_standing_by = dict()
        self.spray_devices_range = dict()


    def spray_enter_standby(self, device, enable = True):
        self.spray_devices_standing_by[device] = enable


    def get_spray_action_state(self, slider_pos, on_min, on_max):
        if on_min < slider_pos < on_max:
            return SprayAction.ON
        else:
            return SprayAction.OFF


    def slider_action(self, action = SliderAction.STOP):
        '''
        if action == SliderAction.STOP:
            self.r.hset("slider", "stop", 1.0)
        else:
            self.r.hset("slider", "stop", 0.0)
        '''

        self.r.hset("slider", "move", action.value)
        self.is_running = action != SliderAction.STOP
        if not self.is_running:
            self.all_spray_off()
            self.is_processing_command = False

    def move_to(self, target):
        if self.min_pos <= target <= self.max_pos:
            pass
        else:
            rospy.logerr("target pos out of range. Stopping")
            self.slider_action(SliderAction.STOP)
            return False
        print('slider_' + self.name + '_laser_distance')
        current_pos = float(self.r.get('slider_' + self.name + '_laser_distance'))

        rospy.loginfo("current_pos: {}".format(current_pos))
        if self.min_pos < current_pos < self.max_pos:
            pass
        else:
            rospy.logerr("current pos out of range. Stopping")
            return False
        if not self.is_processing_command:  # first time; initiate action
            # fetch spray range
            self.register_spray_devices_range()
            if current_pos > target:
                action = SliderAction.DOWN
            else:
                action = SliderAction.UP
            self.slider_action(action=action)
            self.is_processing_command = True
            self.processing_action = action
        else:
            self.slider_action(SliderAction.NONE)
            rospy.loginfo("target: {}".format(target))
            rospy.loginfo("direction: {}".format(self.processing_action))
            # enable spray if needed
            self.process_painting_state_matrix(current_pos)

            # check if arrived within tolerance
            if target - self.tolerance_mm < current_pos < target + self.tolerance_mm:
                self.slider_action(action=SliderAction.STOP)
                rospy.loginfo("slider reached target")
                self.is_processing_command = False
                self.processing_action = None
                # done
            elif self.processing_action == SliderAction.DOWN and self.min_pos - self.tolerance_mm < current_pos < self.min_pos + self.tolerance_mm:
                self.slider_action(action=SliderAction.STOP)
                rospy.logwarn("reached min pos")
                self.is_processing_command = False
                self.processing_action = None
            elif self.processing_action == SliderAction.UP and self.max_pos - self.tolerance_mm < current_pos < self.max_pos + self.tolerance_mm:
                self.slider_action(action=SliderAction.STOP)
                rospy.logwarn("reached max pos")
                self.is_processing_command = False
                self.processing_action = None

        return self.is_processing_command

    def spray_action(self, device, action = SprayAction.OFF):
        if float(self.r.get("spray_main_switch")) < 0.5:
            action = SprayAction.OFF
        self.r.hset("slider", device.value, action.value)

    def all_spray_off(self):

        self.spray_action(device=SprayDevice.HIGH, action=SprayAction.OFF)
        self.spray_action(device=SprayDevice.MID, action=SprayAction.OFF)
        self.spray_action(device=SprayDevice.LOW, action=SprayAction.OFF)

    def all_off(self):
        self.all_spray_off()
        self.slider_action(SliderAction.STOP)

    def get_position(self):
        try:
            return float(self.r.get('slider_' + self.name + '_position_laser'))
        except:
            return None

    def process_painting_state_matrix(self, slider_pos):
        # turn on or turn off device according to current slider position
        devices_key = ['spray_high', 'spray_mid', 'spray_low']
        for key in devices_key:
            spray_min_pos, spray_max_pos= self.spray_devices_range[key]
            action = self.get_spray_action_state(slider_pos, spray_min_pos, spray_max_pos)
            device = spray_device_from_key(key)
            if self.spray_devices_standing_by[device]:
                if device is not None:
                    print(device, action)
                    self.spray_action(device=device, action=action)
        return action

    def register_spray_devices_range(self):
        # TODO later not to hard code the device names
        devices_key = ['spray_high', 'spray_mid', 'spray_low']
        for key in devices_key:
            self.spray_devices_range[key] = (
            float(self.r.hget(key + "_range", 'min')), float(self.r.hget(key + "_range", 'max')))
        print(self.spray_devices_range)
        print(self.spray_devices_standing_by)

class AutoPaint():
    def __init__(self):
        self.min_spacing_mm = 450
        self.tolerance_distance = 25  # mm
        self.walls = None
        self.is_walls_available = False
        loop_rate = rospy.Rate(10)  # 1Hz in gazebo
        self.current_stage = 'read_flow'  # read_flow
        self.sub_lines = rospy.Subscriber('/line_segments', LineSegmentList, self.cbGetLines, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.step_t0 = None
        self.step_d0 = None
        self.step_num = None
        self.steps = None
        self.loops = dict()
        self.r = init_redis(restore_default=False)
        self.slider = SliderPaint(redis_conn=self.r)
        # loop: name: {step_start_num, step_end_num}

        rospy.on_shutdown(self.fnShutDown)
        while not rospy.is_shutdown():
            if self.is_walls_available:
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
                j = i + 1
                while j < len(self.steps):
                    another_step = self.steps[j]
                    another_step_tmp = another_step.split(';')
                    if another_step_tmp[0] == 'loop' and another_step_tmp[1] == 'end' and another_step_tmp[
                        2] == loop_name:
                        loop_step_end_num = j
                        break
                    j += 1
                if loop_step_end_num < 0:  # TODO error
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
        print("walls: ")
        print(walls)
        #if(wal)
        return walls

    def process(self):
        step_keep_running = False
        #if self.is_walls_available and len(self.walls) == 0:
        #    return True
        print(self.get_walls())
        if self.current_stage == 'debug':
            print(self.get_walls(order_by_angle=90))
            return False
        elif self.current_stage == 'bye':
            return True
        elif self.current_stage == 'read_flow':
            try:
                file_name = str(sys.argv[1])
                file_path = "/home/aic/aic_ws/src/laser_line_extraction/scripts/" + file_name
                file1 = open(file_path, 'r')
            except:
                file1 = open('aicode.txt', 'r')
                pass

            ## file1 = open('aicode_rotate.txt', 'r')
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
                pass  # proceed to next step
            else:
                tmp = step.split(';')
                if tmp is None or len(tmp) == 0:  # todo validate more later
                    rospy.logerr('parse step error. Step#{}'.format(self.step_num))
                    return False  # exit
                command = tmp[0]
                if command[0:7] == 'blind_x': #blind_x3000
                    move_time = str(command[7:])
                    move_time = int(move_time)
                    print("move_time: " + str(move_time))
                    self.fnblind_move_x(move_time)
                elif command[0:7] == 'blind_y':
                    move_time = str(command[7:])
                    move_time = int(move_time)
                    print("move_time: " + str(move_time))
                    self.fnblind_move_y(move_time)
                elif command == 'spray_high_on':
                    self.onoff_high_spray(1)
                elif command == 'spray_high_off':
                    self.onoff_high_spray(0)
                elif command == 'spray_low_on':
                    self.onoff_low_spray(1)
                elif command == 'spray_low_off':
                    self.onoff_low_spray(0)
                elif command == 'reset_encoder':
                    self.fnreset_encoder(1)
                elif command == 'spray_mid_off':
                    self.onoff_mid_spray(0)
                elif command == 'spray_mid_on':
                    self.onoff_mid_spray(1)                    
                elif command == 'wait':
                    ms = float(tmp[1])
                    if self.step_t0 is None:
                        self.step_t0 = rospy.get_time()
                    else:
                        remaining = round((self.step_t0 + ms / 1000) - rospy.get_time(), 1)
                        if remaining > 0:
                            rospy.loginfo('waiting..remaining: {}'.format(remaining))
                            step_keep_running = True
                        else:
                            try:
                                # rospy.loginfo(tmp[2])
                                print(tmp[2])
                            except:
                                pass
                elif command == 'start':
                    if tmp[1] == 'slider':
                        if tmp[2] == 'move_to':
                            target = float(tmp[3])
                            self.set_slider_move_distance(target) #For paintbot, command this and set redis
                            try:
                                ms = float(tmp[4])
                            except:
                                ms = self.slider.default_slider_move_timeout_ms
                            if self.step_t0 is None:
                                self.step_t0 = rospy.get_time()
                            else:
                                remaining = round((self.step_t0 + ms / 1000) - rospy.get_time(), 1)
                                if remaining > 0:
                                    rospy.loginfo('remaining {}'.format(remaining))
                                    step_keep_running = self.slider.move_to(target)  # true if arrived target/min/end
                                else:
                                    rospy.logerr("slider move_to timeout.")
                                    self.slider.slider_action(SliderAction.STOP)
                                    step_keep_running = False
                        elif tmp[2] == 'spray_when_move':
                            device_id = int(tmp[3])
                            enable = False
                            try:
                                enable = int(tmp[4]) > 0
                            except:
                                pass
                            device = spray_device_from_id(device_id)
                            print(self.slider.spray_devices_standing_by)
                            if device is not None:
                                self.slider.spray_enter_standby(device=device, enable=enable)

                elif command == 'adjust':
                    global move_x_dir
                    global move_y_dir
                    if tmp[1] == 'distance':
                        #check_distance_wall = True
                        if int(tmp[3]) == 0 or int(tmp[3]) == 180:
                            move_x_dir = True
                            move_y_dir = False
                        elif int(tmp[3]) == 90 or int(tmp[3]) == -90: 
                            move_x_dir = False
                            move_y_dir = True  

                        walls = self.get_walls(float(tmp[3]))
                        target_distance = float(tmp[4])
                        angle, distance = walls[0]

                        if tmp[2] == 'start':
                            target_distance = -1 * target_distance
                            if self.step_d0 is None:
                                self.step_d0 = distance
                            target_distance += self.step_d0

                        if target_distance < self.min_spacing_mm:
                            target_distance = self.min_spacing_mm

                        rospy.loginfo('target distance to wall: {}'.format(target_distance))
                        rospy.loginfo('current distance to wall: {}'.format(distance))
                        if abs(distance - target_distance) > self.tolerance_distance:  # mm
                            # assuming y direction
                            tx = math.cos(math.pi * float(tmp[3]) / 180) * (distance - target_distance) / 100  #sin
                            ty = math.sin(math.pi * float(tmp[3]) / 180) * (distance - target_distance) / 100  #cos
                            self.fnGoStraight(x=tx, y=ty)
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
                            self.fnTurn(-1 * abs(angle - target) * math.pi / 180)
                            step_keep_running = True
                        elif (angle - target) < - 1:
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
                        self.step_num, _ = self.loops[tmp[2]]  # goto step
                    elif tmp[1] == 'break_if':
                        # check condition, if met, go to end
                        broken_loop = False
                        print('break_if')
                        if tmp[3] == 'orientation':
                            walls = self.get_walls(float(tmp[5]))
                            angle, distance = walls[0]
                            target = float(tmp[6])
                            rospy.loginfo('current angle to wall: {}'.format(angle))
                            if angle < target - 1 or angle > target + 1:
                                pass  # do nothing  #proceed to next step
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
        start_points_x = []
        start_points_y = []
        end_points_x = []
        end_points_y = []
        global move_x_dir
        global move_y_dir

        #global global_walls

        if len(lines_msg.line_segments) > 0:
            for line in lines_msg.line_segments:
                angle_deg = line.angle * 180 / math.pi
                if angle_deg > 175:
                    angle_deg = -180.0
                angle_deg = round(angle_deg)
                distance_cm = round(100 * line.radius) #460
                start_point_x = line.start[0]
                start_point_y = line.start[1]
                end_point_x = line.end[0]
                end_point_y = line.end[1]
                
                #Move x dir, y+ - y-
                #Move y dir, x+ - x-
                found = False
                merge_cm = 5
                merge_deg = 5
                for i in range(len(angles)):
                    check_angle = angles[i]
                    check_distance = distances[i]
                    if check_angle- merge_deg < angle_deg < check_angle + merge_deg:
                        #found = True
                        if move_x_dir:
                            if (start_point_y > 0.0 and end_point_y < 0.0):
                                angles.pop(i)
                                distances.pop(i)
                                found = False
                                break
                            else:
                                found = True
                        elif move_y_dir:
                            if (start_point_x > 0.0 and end_point_x < 0.0):
                                angles.pop(i)
                                distances.pop(i)
                                found = False
                                break
                            else:
                                found = True                                        
    

                if not found:
                    angles.append(angle_deg)
                    distances.append(distance_cm)  # mm
                    
                    #print("angle" + str(angle_deg))
                    #print("distance" + str(distance_cm))
                    #print("y+" + str(start_point_y))
                    #print("y-" + str(end_point_y))
                    
                    
        distances = np.array(distances) * 10
        walls = zip(angles, distances)
        self.walls = sorted(walls, key=lambda x: abs(x[1]), reverse=False)  # sort by distance by default

        if not self.is_walls_available:
            self.is_walls_available = True


    def onoff_high_spray(self, onoff):
        if onoff == 1:
            self.slider.spray_action(device=SprayDevice.HIGH, action=SprayAction.ON)
        elif onoff == 0:
            self.slider.spray_action(device=SprayDevice.HIGH, action=SprayAction.OFF)

    def onoff_low_spray(self, onoff):
        if onoff == 1:
            self.slider.spray_action(device=SprayDevice.LOW, action=SprayAction.ON)
        elif onoff == 0:
            self.slider.spray_action(device=SprayDevice.LOW, action=SprayAction.OFF)


    def onoff_mid_spray(self, onoff):
        if onoff == 1:
            self.slider.spray_action(device=SprayDevice.MID, action=SprayAction.ON)
        elif onoff == 0:
            self.slider.spray_action(device=SprayDevice.MID, action=SprayAction.OFF)


    def onoff_servo(self,onoff):
        if onoff == 1:
            self.r.set("servo_1", 1)
        elif onoff == -1:
            self.r.set("servo_2", 1)
        elif onoff == 0:
            self.r.set("servo_1", 0)
            self.r.set("servo_2", 0)


    def fnStop(self):
        self.fnGoStraight(0, 0, 0)


    def fnblind_move_x(self,move_time):
        twist = Twist()
        twist.linear.x = 0.3
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        last_time_milliseconds = int(round(time.time()*1000))
        while((int(round(time.time()*1000)) - last_time_milliseconds) < move_time):
            self.pub_cmd_vel.publish(twist)

    def fnblind_move_y(self,move_time):
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = -0.3
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        last_time_milliseconds = int(round(time.time()*1000))
        while((int(round(time.time()*1000)) - last_time_milliseconds) < move_time):
            self.pub_cmd_vel.publish(twist)

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


    def fnreset_encoder(self,reset):
        if reset:
            self.r.set("encoder_reset", 1)
            time.sleep(0.5)
            self.r.set("encoder_reset", 0)


    def set_slider_move_distance(self, target):
        slider_cur_pos = float(self.r.get("slider_main_laser_distance"))
        slider_target_distance = float(target)
        remain_distance = abs(int(target - slider_cur_pos))
        self.r.hset("slider", "move_distance", remain_distance)
        rospy.loginfo("slider distance remaining: " + str(remain_distance))


    def fnGoStraight(self, x=0, y=0, z=0, limit=0.3, fixed_speed=False):
        global move_y_dir

        if not fixed_speed:
            Kp = 0.8
            x = Kp * x
            y = Kp * y
            z = Kp * z
            x = self.clamp(x, -limit, limit)
            y = self.clamp(y, -limit, limit)
            z = self.clamp(z, -limit, limit)
        twist = Twist()
        twist.linear.x = x*1.5
        twist.linear.y = y*1.5
        twist.linear.z = z
        twist.angular.x = 0
        twist.angular.y = 0
        if move_y_dir:
            twist.angular.z = float(self.r.hget("laser_parallel","angular_vel"))
        else:
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

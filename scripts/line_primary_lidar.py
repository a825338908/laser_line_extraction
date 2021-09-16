#!/usr/bin/env python

from laser_line_extraction.msg import LineSegment, LineSegmentList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import sensor_msgs.msg
import math
import redis

red = redis.Redis(host='localhost', port=6379)
offset_lines_pub = rospy.Publisher('offset_new_line1', LineSegmentList, queue_size=1)
offset_lines_marker_pub = rospy.Publisher('offset_new_line1_marker', Marker, queue_size=1)


def redis_value_init(init):
    if init:
        red.hset("line", "line1_x_offset", 0.0)
        red.hset("line", "line1_y_offset", 0.0)
        red.hset("line", "line1_remove_angle_0", 0)
        red.hset("line", "line1_remove_angle_1", 0)
        red.hset("line", "remove_line", 0)


def marker(line_msg):
    marker = Marker()
    marker.ns = "offset_line_node"
    marker.id = 0;
    marker.type = Marker.LINE_LIST
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    for line in line_msg.line_segments:
        p_start = Point()
        p_start.x = line.start[0]
        p_start.y = line.start[1]
        p_start.z = 0
        marker.points.append(p_start)
        p_end = Point()
        p_end.x = line.end[0]
        p_end.y = line.end[1]
        marker.points.append(p_end)
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    offset_lines_marker_pub.publish(marker)
               


def offsetLines(lines_msg):
    #print(type(lines_msg.line_segments))
    line1_remove_angle_0 = int(red.hget("line", "line1_remove_angle_0"))
    line1_remove_angle_1 = int(red.hget("line", "line1_remove_angle_1"))
    remove_line = int(red.hget("line", "remove_line"))
    offset = 30   


    idx_list = []
    idx = 0
    for line in lines_msg.line_segments:
        x_dir_offset = float(red.hget("line", "line1_x_offset"))
        y_dir_offset = float(red.hget("line", "line1_y_offset"))  
        degree = line.angle * 180 / math.pi   
        #print("    " +str(degree))
        if  -30 < degree < 30 or 150 < degree < 210: #0, 3.14
            tmp_list_start = list(line.start)
            tmp_list_start[0] = tmp_list_start[0] + x_dir_offset
            tmp_list_start[1] = tmp_list_start[1] + y_dir_offset
            line.start = tuple(tmp_list_start)

            tmp_list_end = list(line.end)
            tmp_list_end[0] = tmp_list_end[0] + x_dir_offset
            tmp_list_end[1] = tmp_list_end[1] + y_dir_offset
            line.end = tuple(tmp_list_end)     
            line.radius = line.radius + x_dir_offset
            
        elif 60 < degree < 120 or -120 < degree < -60: # 1.57 
            tmp_list_start = list(line.start)
            tmp_list_start[0] = tmp_list_start[0] + x_dir_offset
            tmp_list_start[1] = tmp_list_start[1] + y_dir_offset
            line.start = tuple(tmp_list_start)

            tmp_list_end = list(line.end)
            tmp_list_end[0] = tmp_list_end[0] + x_dir_offset
            tmp_list_end[1] = tmp_list_end[1] + y_dir_offset
            line.end = tuple(tmp_list_end)
            line.radius = line.radius + y_dir_offset 
      
        if remove_line == 1:
            if line1_remove_angle_0 - offset < degree <  line1_remove_angle_0 + offset:
                if len(idx_list) < 1:
                    idx_list.append(idx)
                else:
                    idx_list.append(idx)                  
            elif line1_remove_angle_1 - offset < degree <  line1_remove_angle_1 + offset:
                if len(idx_list) < 1:
                    idx_list.append(idx)
                else:
                    idx_list.append(idx)              
        idx = idx + 1

    for i, value in enumerate(idx_list):
        value = value - i
        lines_msg.line_segments.pop(value)
       
    

    offset_lines_pub .publish(lines_msg)   
    marker(lines_msg)         
          

def listener():
    rospy.init_node('sub_line_node_1', anonymous=True)
    sub_lines = rospy.Subscriber('/line1/line_segments', LineSegmentList, offsetLines, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    listener()
    redis_value_init(False)
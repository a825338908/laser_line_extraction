#!/usr/bin/env python

from laser_line_extraction.msg import LineSegment, LineSegmentList
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import sensor_msgs.msg
import math
import redis

red = redis.Redis(host='localhost', port=6379)
line_combine_pub = rospy.Publisher('line/combine', LineSegmentList, queue_size=1)
offset_lines_marker_pub = rospy.Publisher('line_combine_marker', Marker, queue_size=1)

tmp_line = LineSegmentList()

def marker(line_msg):
    marker = Marker()
    marker.ns = "combine_line_node"
    marker.id = 0;
    marker.type = Marker.LINE_LIST
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
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


def line_combine(line_msg):
    global tmp_line # line0
    line1 = line_msg # line1
    offset_line0 = tmp_line
    combine_line = LineSegmentList()
    combine_line.header = line_msg.header   
    combine_line.line_segments = tmp_line.line_segments + line1.line_segments
    line_combine_pub.publish(combine_line)
    marker(combine_line)
    




def sub_offset_line(lines_msg):
    #print(lines_msg.line_segments[0].radius)
    global tmp_line #line0 offset
    tmp_line = lines_msg
    sub_offset_lines = rospy.Subscriber('/offset_new_line1', LineSegmentList, line_combine, queue_size=1)         
        
          

def listener():
    rospy.init_node('line_combine', anonymous=True)
    sub_offset_lines = rospy.Subscriber('/offset_new_line0', LineSegmentList, sub_offset_line, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()

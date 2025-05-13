#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from robot_api.base import Base  # 确保你有 robot_api 并安装正确

MOVE_DISTANCE = 0.5
TURN_ANGLE = math.radians(30)

base = None  # 全局变量，表示底盘控制对象

def create_marker(name, description, position, color, callback):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = name
    int_marker.description = description
    int_marker.scale = 0.4
    int_marker.pose.position = position

    box_marker = Marker()
    box_marker.type = Marker.CUBE
    box_marker.scale.x = 0.3
    box_marker.scale.y = 0.3
    box_marker.scale.z = 0.3
    box_marker.color.r = color[0]
    box_marker.color.g = color[1]
    box_marker.color.b = color[2]
    box_marker.color.a = 1.0

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.markers.append(box_marker)

    int_marker.controls.append(control)

    return int_marker

def process_feedback(feedback):
    global base
    if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
        return

    name = feedback.marker_name
    rospy.loginfo(f"{name} clicked")

    if name == "forward":
        base.go_forward(MOVE_DISTANCE)
    elif name == "backward":
        base.go_forward(-MOVE_DISTANCE)
    elif name == "left":
        base.turn(TURN_ANGLE)
    elif name == "right":
        base.turn(-TURN_ANGLE)

def main():
    global base
    rospy.init_node("base_int_marker_demo")
    base = Base()
    server = InteractiveMarkerServer("base_marker_server")

    markers = [
        ("forward", "Move Forward", Point(0.6, 0.0, 0.2), (0.0, 1.0, 0.0)),    # 绿色
        ("backward", "Move Backward", Point(-0.6, 0.0, 0.2), (1.0, 0.0, 0.0)), # 红色
        ("left", "Turn Left", Point(0.0, 0.6, 0.2), (0.0, 0.0, 1.0)),          # 蓝色
        ("right", "Turn Right", Point(0.0, -0.6, 0.2), (1.0, 1.0, 0.0)),       # 黄色
    ]

    for name, desc, pos, color in markers:
        marker = create_marker(name, desc, pos, color, process_feedback)
        server.insert(marker, process_feedback)

    server.applyChanges()
    rospy.spin()

if __name__ == "__main__":
    main()

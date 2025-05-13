#!/usr/bin/env python

import rospy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Point
import robot_api
import math

class InteractiveBaseControl(object):
    def __init__(self):
        self.server = InteractiveMarkerServer("base_control")
        self.base = robot_api.Base()
        self._make_marker("go_forward", "Go Forward", Point(0.5, 0, 0.2))
        self._make_marker("turn_left", "Turn Left", Point(0.4, 0.3, 0.2))
        self._make_marker("turn_right", "Turn Right", Point(0.4, -0.3, 0.2))
        self.server.applyChanges()

    def _make_marker(self, name, description, position):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 0.3
        marker.color.a = 1.0
        marker.pose.orientation.w = 1

        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link"
        int_marker.name = name
        int_marker.description = description
        int_marker.pose.position = position
        int_marker.scale = 0.3

        control = InteractiveMarkerControl()
        control.always_visible = True
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.markers.append(marker)

        int_marker.controls.append(control)
        self.server.insert(int_marker, self._handle_click)

    def _handle_click(self, feedback):
        if feedback.event_type == feedback.BUTTON_CLICK:
            if feedback.marker_name == "go_forward":
                self.base.go_forward(0.5)
            elif feedback.marker_name == "turn_left":
                self.base.turn(math.radians(30))
            elif feedback.marker_name == "turn_right":
                self.base.turn(math.radians(-30))

def main():
    rospy.init_node('interactive_base_control')
    InteractiveBaseControl()
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from robot_api import Base
import math

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

base = Base()

def handle_viz_input(input):
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and input.control_name == "f":
        base.go_forward(0.5)
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and input.control_name == "l":
        base.turn(math.pi/6.0)
    if input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK and input.control_name == "r":
        base.turn(-math.pi/6.0)

def main():
    rospy.init_node('interactive_marker_demo')
    wait_for_time()
    server = InteractiveMarkerServer("simple_marker")

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.name = "Drive"
    int_marker.pose.orientation.w = 1

    forward_marker = Marker(type=Marker.CUBE,
                         scale=Vector3(0.45, 0.45, 0.45),
                         color=ColorRGBA(0.0, 0.5, 0.5, 1.0))
    forward_marker.pose.position.x = 1
    forward_marker.pose.orientation.w = 1
    left_marker = Marker(type=Marker.CUBE,
                         scale=Vector3(0.45, 0.45, 0.45),
                         color=ColorRGBA(0.0, 0.5, 0.5, 1.0))
    left_marker.pose.position.y = 1
    left_marker.pose.orientation.w = 1
    right_marker = Marker(type=Marker.CUBE,
                         scale=Vector3(0.45, 0.45, 0.45),
                         color=ColorRGBA(0.0, 0.5, 0.5, 1.0))
    right_marker.pose.position.y = -1
    right_marker.pose.orientation.w = 1

    fbutton_control = InteractiveMarkerControl(name="f")
    fbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    fbutton_control.always_visible = True
    fbutton_control.markers.append(forward_marker)
    int_marker.controls.append(fbutton_control)

    lbutton_control = InteractiveMarkerControl(name="l")
    lbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    lbutton_control.always_visible = True
    lbutton_control.markers.append(left_marker)
    int_marker.controls.append(lbutton_control)

    rbutton_control = InteractiveMarkerControl(name="r")
    rbutton_control.interaction_mode = InteractiveMarkerControl.BUTTON
    rbutton_control.always_visible = True
    rbutton_control.markers.append(right_marker)
    int_marker.controls.append(rbutton_control)

    server.insert(int_marker, handle_viz_input)
    server.applyChanges()
    rospy.spin()

if __name__ == "__main__":
    main()

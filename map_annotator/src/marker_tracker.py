#!/usr/bin/env python

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker

def marker_helper(name):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = name
    int_marker.description = name
    int_marker.pose.position.z = 0.05

    arrow = Marker()
    arrow.type = Marker.ARROW
    arrow.pose.orientation.w = 1
    arrow.pose.position.z = 0.05
    arrow.scale.x = 0.50
    arrow.scale.y = 0.10
    arrow.scale.z = 0.10
    arrow.color.r = 0.0
    arrow.color.g = 0.5
    arrow.color.b = 0.5
    arrow.color.a = 1.0

    rotate = InteractiveMarkerControl()
    rotate.orientation.w = 1;
    rotate.orientation.x = 0;
    rotate.orientation.y = 1;
    rotate.orientation.z = 0;

    rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate.orientation_mode = InteractiveMarkerControl.FIXED

    move = InteractiveMarkerControl()
    move.orientation.w = 1;
    move.orientation.x = 0;
    move.orientation.y = 1;
    move.orientation.z = 0;
    move.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    move.orientation_mode = InteractiveMarkerControl.INHERIT


    rotate.always_visible = True
    move.markers.append(arrow)
    move.always_visible = True
    int_marker.controls.append(move)
    int_marker.controls.append(rotate)
    return int_marker

class MarkerTracker():

    def __init__(self, annotator):
        self.server = InteractiveMarkerServer("map_annotator")
        self.ann = annotator
        self.markers = {}
        for name, pose in self.ann.list().iteritems():
            self.create(name, pose=pose)
        
    def create(self, name, pose=None):
        marker = marker_helper(name)
        # keep track of marker
        self.markers[name] = marker

        if pose is None:
            pose = marker.controls[0].markers[0].pose
        else:
            marker.pose = pose
        self.ann.save(name, pose=pose)

        # set callback for marker with server
        self.server.insert(marker, self._update_poses)
        self.server.applyChanges()


    def delete(self, name):
        # TODO : Delete marker associated with name
        self.ann.delete(name)
        # remove marker with server
        self.server.erase(name)
        self.server.applyChanges()

    def goto(self, name):
        self.ann.goto(name)

    def _update_poses(self, input):
        pose = input.pose
        #print("update_poses", input.marker_name, pose)
        self.ann.save(input.marker_name, pose=pose)


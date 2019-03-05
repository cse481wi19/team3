#! /usr/bin/env python

import robot_api
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import math
from copy import deepcopy

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MarkerTracker():
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

    def get_id(self, mid):
        for m in self.markers:
            if m.id == mid:
                return m
        return None

class PoseTracker():
    def __init__(self):
        self.pose = Pose()

    def callback(self, msg):
        self.pose = msg

def main():
    rospy.init_node('whiteboard_box')
    wait_for_time()

    whiteboard_width = 0.5
    whiteboard_height = 0.5
    whiteboard_depth = 0.1
    br = tf.TransformBroadcaster()
    markers = MarkerTracker()
    sub = rospy.Subscriber('whiteboard_tags', MarkerArray,
            callback=markers.callback) # Subscribe to AR tag poses, use reader.callback
    pose = PoseTracker()
    sub = rospy.Subscriber('plane_pose', Pose,
            callback=pose.callback) # Subscribe to AR tag poses, use reader.callback

    pub = rospy.Publisher('whiteboard_region', Marker, queue_size=1)
    
    r = rospy.Rate(2)
    def makeMarker(pose):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = whiteboard_width
        marker.scale.y = whiteboard_height
        marker.scale.z = whiteboard_depth
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.pose.orientation = deepcopy(pose.pose.orientation)
        marker.header.frame_id = 'base_link'
        return marker
    whiteboard_marker = makeMarker(pose)

    while 1:
        r.sleep()
        if len(markers.markers) == 0:
            print "No AR markers found"
            continue
        reference_marker = markers.get_id(16)
        if reference_marker is None:
            print "What you do?"
            continue
        whiteboard_marker = makeMarker(pose)
        whiteboard_marker.pose.position = deepcopy(reference_marker.pose.position)
        whiteboard_marker.pose.position.y -= whiteboard_width / 2.0
        whiteboard_marker.pose.position.z += whiteboard_height / 2.0
        print(whiteboard_marker)

        pub.publish(whiteboard_marker)
        br.sendTransform((pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                         (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w),
                         rospy.Time.now(),
                         "whiteboard",
                         "odom")


if __name__ == '__main__':
    main()

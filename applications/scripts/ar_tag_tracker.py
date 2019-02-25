#! /usr/bin/env python

import robot_api
import rospy
import tf
import tf.transformations as tft
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

AR_POSE_TOPIC = '/ar_pose_marker'

def makeMarker(m):
    """Creates a marker object from an AlvarMarker object"""
    res = Marker()
    res.id = m.id
    res.header = m.header
    res.type = Marker.CUBE
    res.action = Marker.ADD
    res.pose = m.pose.pose

    res.color.a = 1
    res.color.r = 0.5
    res.color.b = 0.5
    res.color.g = 0.0
    res.scale.x = 0.05
    res.scale.y = 0.08
    res.scale.z = 0.02

    return res

class ArTagTracker(object):
    def __init__(self, pub):
        self.markers = []
        self.pub = pub

    def callback(self, msg):
        new_markers = []
        for m in msg.markers:
            new_markers.append(m)
        for m in self.markers:
            if not (m.id in [mark.id for mark in new_markers]):
                new_markers.append(m)
        self.markers = new_markers
        self.publish()

    def publish(self):
        print(self.markers)
        msg = MarkerArray()
        msg.markers = [makeMarker(m) for m in self.markers]
        self.pub.publish(msg)

    def get_id(self, tagid):
        for m in self.markers:
            if m.id == tagid:
                return m
        return None

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    rospy.init_node('ar_tag_tracker')
    wait_for_time()

    pub = rospy.Publisher('whiteboard_tags', MarkerArray, queue_size=10)
    reader = ArTagTracker(pub)
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
            callback=reader.callback) # Subscribe to AR tag poses, use reader.callback

    rospy.spin()


if __name__ == '__main__':
    main()

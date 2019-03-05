#! /usr/bin/env python

import robot_api
import rospy
import tf
import tf.transformations as tft
import numpy as np
from copy import deepcopy

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

class ArTag(object):
    """Responsible for tracking the pose of a single AR tag"""
    def __init__(self, tag, n=10):
        self.id = tag.id
        self.tag = deepcopy(tag)
        self.average_pose = deepcopy(tag.pose)
        self.pose_filter = []
        self.pose_filter.append(self.average_pose)
        self.n = n
        self.next_index = 1

    def update(self, m):
        self.tag = deepcopy(m)
        new_pose = m.pose
        if len(self.pose_filter) < self.n:
            self.pose_filter.append(new_pose)
        else:
            self.pose_filter[self.next_index] = new_pose
        self.next_index += 1
        self.next_index %= self.n
        self.update_average_pose()
        self.tag.pose = self.average_pose

    def update_average_pose(self):
        res_pose = PoseStamped()
        for p in self.pose_filter:
            res_pose.pose.position.x += p.pose.position.x
            res_pose.pose.position.y += p.pose.position.y
            res_pose.pose.position.z += p.pose.position.z
            res_pose.pose.orientation.x += p.pose.orientation.x
            res_pose.pose.orientation.y += p.pose.orientation.y
            res_pose.pose.orientation.z += p.pose.orientation.z
            res_pose.pose.orientation.w += p.pose.orientation.w
        res_pose.pose.position.x /= len(self.pose_filter)
        res_pose.pose.position.y /= len(self.pose_filter)
        res_pose.pose.position.z /= len(self.pose_filter)
        res_pose.pose.orientation.x /= len(self.pose_filter)
        res_pose.pose.orientation.y /= len(self.pose_filter)
        res_pose.pose.orientation.z /= len(self.pose_filter)
        res_pose.pose.orientation.w /= len(self.pose_filter)
        self.average_pose = res_pose

    def get_marker(self):
        return self.tag

class ArTagTracker(object):
    def __init__(self, pub, n=10):
        self.markers = []
        self.marker_trackers = {}
        self.pub = pub
        self.n = n

    def callback(self, msg):
        for m in msg.markers:
            if self.marker_trackers.get(m.id) is not None:
                self.marker_trackers[m.id].update(m)
            else:
                self.marker_trackers[m.id] = ArTag(m, self.n)
        """
        new_markers = []
        for m in msg.markers:
            new_markers.append(m)
        for m in self.markers:
            if not (m.id in [mark.id for mark in new_markers]):
                new_markers.append(m)
        """
        self.markers = [m.get_marker() for m in self.marker_trackers.values()]
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

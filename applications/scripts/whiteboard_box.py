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

def convertPose(pose, tfl):
    try:
        (trans, rot) = tfl.lookupTransform('/odom', '/base_link', rospy.Time(0))
    except:
        return pose
    odom_mat = tfl.fromTranslationRotation(trans, rot)
    po_mat = tfl.fromTranslationRotation([pose.position.x,
                                          pose.position.y,
                                          pose.position.z],
                                         [pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w])
    res_mat = np.dot(odom_mat, po_mat)
    resTrans = tft.translation_from_matrix(res_mat)
    resRot = tft.quaternion_from_matrix(res_mat)
    res = Pose(Point(resTrans[0], resTrans[1], resTrans[2]),
               Quaternion(resRot[0], resRot[1], resRot[2], resRot[3]))
    return res

def main():
    rospy.init_node('whiteboard_box')
    wait_for_time()

    whiteboard_width = 0.5
    whiteboard_height = 0.5
    whiteboard_depth = 0.1
    br = tf.TransformBroadcaster()
    lf = tf.TransformListener()
    markers = MarkerTracker()
    sub = rospy.Subscriber('whiteboard_tags', MarkerArray,
            callback=markers.callback) # Subscribe to AR tag poses, use reader.callback
    pose = PoseTracker()
    sub = rospy.Subscriber('plane_pose', Pose,
            callback=pose.callback) # Subscribe to whiteboard plane, use pose.callback

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
        reference_marker = markers.get_id(14)
        if reference_marker is None:
            print "What you do?"
            continue
        whiteboard_marker = makeMarker(pose)
        whiteboard_marker.pose.position = deepcopy(reference_marker.pose.position)
        whiteboard_frame = deepcopy(whiteboard_marker.pose)
        whiteboard_marker.pose.position.y -= whiteboard_width / 2.0
        whiteboard_marker.pose.position.z += whiteboard_height / 2.0

        pub.publish(whiteboard_marker)
        converted = convertPose(whiteboard_frame, lf)
        old_rot = [converted.orientation.x,
            converted.orientation.y, converted.orientation.z,
            converted.orientation.w]
        angles = tft.euler_from_quaternion([converted.orientation.x,
            converted.orientation.y, converted.orientation.z,
            converted.orientation.w])
        print("Angles:", angles)
        rot = tft.quaternion_from_euler(0, 0, 0)
        res_orientation = tft.quaternion_multiply(rot, old_rot)
        converted.orientation = Quaternion(res_orientation[0], res_orientation[1], res_orientation[2], res_orientation[3])

        print("whiteboard:")
        print(converted)
        print("marker:")
        print(reference_marker.pose)
        print("======")
        br.sendTransform((converted.position.x, converted.position.y, converted.position.z),
                         (converted.orientation.x, converted.orientation.y, converted.orientation.z, converted.orientation.w),
                         rospy.Time.now(),
                         "whiteboard",
                         "odom")


if __name__ == '__main__':
    main()

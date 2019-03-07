#! /usr/bin/env python

import robot_api
import rospy
import tf
import tf.transformations as tft
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
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

    corners = {14:"bottom left"}

    br = tf.TransformBroadcaster()
    tfl = tf.TransformListener()
    markers = MarkerTracker()
    sub = rospy.Subscriber('whiteboard_tags', MarkerArray,
            callback=markers.callback) # Subscribe to AR tag poses, use reader.callback
    pose = PoseTracker()
    sub = rospy.Subscriber('plane_pose', Pose,
            callback=pose.callback) # Subscribe to whiteboard plane, use pose.callback

    pub = rospy.Publisher('whiteboard_region', Marker, queue_size=1)
    
    r = rospy.Rate(30)

    def makeMarker(orientation):
        marker = Marker()
        marker.type = Marker.CUBE
        marker.scale.x = whiteboard_width
        marker.scale.y = whiteboard_height
        marker.scale.z = whiteboard_depth
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        marker.pose.orientation = deepcopy(orientation)
        marker.header.frame_id = 'base_link'
        return marker

    whiteboard_marker = makeMarker(pose)

    def stabilizeOrientation(orientation):
        def expandQuat(q):
            return [q.x, q.y, q.z, q.w]

        def rotateVectorByQuat(v, q):
            rot = tft.quaternion_matrix(expandQuat(q))
            return np.dot(rot, [v[0], v[1], v[2], 1])[0:3]

        """Returns the quaternion obtained by rotating 'orientation' so that x
           is perpendicular to the robot's vertical direction and in the
           plane of the whiteboard. In this case, x is positive to the right"""
        old_x = [1, 0, 0]
        old_y = [0, 1, 0]
        old_z = [0, 0, 1]
        new_x = rotateVectorByQuat(old_x, orientation)
        new_y = rotateVectorByQuat(old_y, orientation)
        new_z = rotateVectorByQuat(old_z, orientation)
        reflect = np.identity(4)

        if new_z[0] < 0:
            reflect = tft.reflection_matrix([0, 0, 0], new_z)

        goal_x = tft.unit_vector(np.cross(new_z, old_z))
        angle = np.arccos(np.dot(new_x, goal_x))
        rot = tft.rotation_matrix(angle, new_z)
        
        res_x = np.dot(rot, [new_x[0], new_x[1], new_x[2], 1])[0:3]

        if res_x[1] > 0:
            rot = tft.rotation_matrix(angle + np.pi, new_z)

        old_quat = expandQuat(pose.pose.orientation)
        rot_quat = tft.quaternion_from_matrix(rot)
        reflect_quat = tft.quaternion_from_matrix(reflect)
        new_quat = tft.quaternion_multiply(tft.quaternion_multiply(rot_quat, old_quat), reflect_quat)
        return Quaternion(new_quat[0], new_quat[1], new_quat[2], new_quat[3])

    while 1:
        r.sleep()
        orientation = stabilizeOrientation(pose.pose.orientation)
        if len(markers.markers) == 0:
            print "No AR markers found"
            continue
        reference_marker = markers.get_id(14)
        if reference_marker is None:
            print "What you do?"
            continue
        whiteboard_marker = makeMarker(orientation)
        whiteboard_marker.pose.position = deepcopy(reference_marker.pose.position)
        whiteboard_ps = PoseStamped()
        whiteboard_ps.pose.orientation = deepcopy(orientation)
        whiteboard_ps.header.frame_id = "base_link"
        
        try:
            whiteboard_ps = tfl.transformPose('odom', whiteboard_ps)
        except (e):
            continue

        whiteboard_ps.pose.position = deepcopy(reference_marker.pose.position)

        """Shift whiteboard marker position so bottom corner is correctly placed"""
        whiteboard_marker.pose.position.y -= whiteboard_width / 2.0
        whiteboard_marker.pose.position.z += whiteboard_height / 2.0
        pub.publish(whiteboard_marker)
        

        br.sendTransform((whiteboard_ps.pose.position.x, whiteboard_ps.pose.position.y, whiteboard_ps.pose.position.z),
                         (whiteboard_ps.pose.orientation.x, whiteboard_ps.pose.orientation.y, whiteboard_ps.pose.orientation.z, whiteboard_ps.pose.orientation.w),
                         rospy.Time.now(),
                         "whiteboard",
                         "odom")


if __name__ == '__main__':
    main()

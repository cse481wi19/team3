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
        """Rotated the wrong way - go another 180 degrees"""
        rot = tft.rotation_matrix(angle + np.pi, new_z)

    old_quat = expandQuat(orientation)
    rot_quat = tft.quaternion_from_matrix(rot)
    reflect_quat = tft.quaternion_from_matrix(reflect)
    new_quat = tft.quaternion_multiply(tft.quaternion_multiply(rot_quat, old_quat), reflect_quat)
    return Quaternion(new_quat[0], new_quat[1], new_quat[2], new_quat[3])


def makeMarker(dimensions, pose):
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = dimensions[0]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[2]
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.pose = Pose()
    """Shift whiteboard marker position so bottom corner is correctly placed"""
    marker.pose.position.x += dimensions[0] / 2.0
    marker.pose.position.y -= dimensions[1] / 2.0
    marker.pose.position.z += dimensions[2] / 2.0
    marker.header.frame_id = 'whiteboard'
    return marker

class WhiteboardFrameTracker():
    def __init__(self, poseTracker, markerTracker, tags, frame_br, marker_pub, tfl, dimensions):
        self.poseTracker = poseTracker
        self.markerTracker = markerTracker
        self.tags = tags
        self.br = frame_br
        self.tfl = tfl
        self.marker_pub = marker_pub
        self.dimensions = dimensions
        self.localize = True

        """State variables"""
        self.frame_pose = Pose()
        self.orientation_filter = []
        self.orientation_filter_maxsize = 10
        self.orientation_inlier_angle = 0.4 # Approximately pi/8

    def loop(self):
        if self.localize:
            self.updateOrientationFilter()
            self.getFramePosition()
        else:
            self.updateLocalize()
        self.broadcastFrame()
        self.publishMarker()

    def updateLocalize(self):
        """Add a topic to subscribe to a bool that lets us re-localize"""
        pass

    def updateOrientationFilter(self):
        """Uses self.poseTracker to include a new change to the whiteboard
           pose if not an outlier. If there is an update, updates frame_pose"""
        # TODO IMPLEMENT
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose.orientation = stabilizeOrientation(self.poseTracker.pose.orientation)
        try:
            ps = self.tfl.transformPose('odom', ps)
        except:
            return
        res = self.insertOrientation(ps.pose.orientation)
        if res:
            self.frame_pose.orientation = ps.pose.orientation
        else:
            self.frame_pose.orientation = Quaternion()

    def isOutlier(self, orientation):
        def expandQuat(quat):
            return [quat.x, quat.y, quat.z, quat.w]
        orvec = expandQuat(orientation)
        for q in self.orientation_filter:
            qvec = expandQuat(q)
            angle = np.arccos(np.dot(orvec, qvec))
            if angle > self.orientation_inlier_angle:
                return True
        return False

    def insertOrientation(self, orientation):
        if self.isOutlier(orientation):
            self.orientation_filter = []
            return False
        self.orientation_filter.append(orientation)
        return True

    def getFramePosition(self):
        """Uses self.markerTracker to get the current position for the bottom
           left corner of the whiteboard region, and updates frame_pose"""
        # TODO implement
        if (len(self.markerTracker.markers) == 0) or (self.markerTracker.get_id(14) is None):
            self.frame_pose.position = self.poseTracker.pose.position
        else:
            m = self.markerTracker.get_id(14)
            self.frame_pose.position = m.pose.position

    def broadcastFrame(self):
        """Broadcasts the whiteboard frame under odom"""
        self.br.sendTransform((self.frame_pose.position.x, self.frame_pose.position.y, self.frame_pose.position.z),
                         (self.frame_pose.orientation.x, self.frame_pose.orientation.y, self.frame_pose.orientation.z, self.frame_pose.orientation.w),
                         rospy.Time.now(),
                         "whiteboard",
                         "odom")

    def publishMarker(self):
        """Publishes the representation of the whiteboard region"""
        m = makeMarker(self.dimensions, self.frame_pose)
        self.marker_pub.publish(m)
 

def main():
    rospy.init_node('whiteboard_box')
    wait_for_time()

    whiteboard_width = 0.5
    whiteboard_height = 0.5
    whiteboard_depth = 0.1

    corners = {14:"bottom left"}
    dimensions = [whiteboard_width, whiteboard_height, whiteboard_depth]

    br = tf.TransformBroadcaster()
    tfl = tf.TransformListener()
    markers = MarkerTracker()
    sub = rospy.Subscriber('whiteboard_tags', MarkerArray,
            callback=markers.callback) # Subscribe to AR tag poses, use reader.callback
    pose = PoseTracker()
    sub = rospy.Subscriber('plane_pose', Pose,
            callback=pose.callback) # Subscribe to whiteboard plane, use pose.callback

    pub = rospy.Publisher('whiteboard_region', Marker, queue_size=1)
    wft = WhiteboardFrameTracker(pose, markers, corners, br, pub, tfl, dimensions)
    r = rospy.Rate(30)

    while 1:
        r.sleep()
        wft.loop()

if __name__ == '__main__':
    main()

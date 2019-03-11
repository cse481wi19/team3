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
import actionlib
import robot_api.constants as constants

from applications.msg import LocalizeWhiteboardAction

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

class MarkerTracker():
    def __init__(self):
        self.markers = {}

    def callback(self, msg):
        self.markers[msg.id] = msg

    def get_id(self, mid):
        if mid in self.markers.keys():
            return self.markers[mid]
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

def computeTranslation(point, orientation):
    def expandPoint(p):
        return [p.x, p.y, p.z]

    def multScalar(s, p):
        res = []
        for j in p:
            res.append(s * j)
        return res

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
    ref_x = rotateVectorByQuat(old_x, orientation)
    ref_y = rotateVectorByQuat(old_y, orientation)
    ref_z = rotateVectorByQuat(old_z, orientation)

    p = expandPoint(point)
    res_x = multScalar(p[0], ref_x)
    res_y = multScalar(p[1], ref_y)
    res_z = multScalar(p[2], ref_z)

    return Point(res_x[0] + res_y[0] + res_z[0],
                 res_x[1] + res_y[1] + res_z[1],
                 res_x[2] + res_y[2] + res_z[2])

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
    """Shift whiteboard marker position so top corner is correctly placed"""
    marker.pose.position.x += dimensions[0] / 2.0
    marker.pose.position.y += dimensions[1] / 2.0
    marker.pose.position.z += dimensions[2] / 2.0
    marker.header.frame_id = 'whiteboard'
    return marker

class WhiteboardFrameTracker():
    def __init__(self, poseTracker, markerTracker, tags, frame_br, marker_pub, tfl, dimensions):
        self.poseTracker = poseTracker
        self.markerTracker = markerTracker
        self.tags = tags # Map from tag id to whiteboard coordinate.
        self.br = frame_br
        self.tfl = tfl
        self.marker_pub = marker_pub
        self.dimensions = dimensions

        """State variables"""
        self.frame_pose = Pose()
        self.orientation_filter = []
        self.orientation_filter_maxsize = 60
        self.orientation_inlier_angle = 0.2 # Approximately pi/16
        self.pose_time_threshold = 10 # seconds beyond which a pose is stale
        self.pose_filter = []
        self.pose_filter_maxsize = 60
        self.pose_inlier_dist = 0.04 # 10 centimeters

        self.localize()

        """Server setup"""
        self.server = actionlib.SimpleActionServer('localize_whiteboard',
                LocalizeWhiteboardAction, self.execute, False)
        self.server.start()

        """Timer callback setup"""
        self.timer = rospy.Timer(rospy.Duration(1.0), self.loop)
        print("Initialized")

    def localize(self):
        localized = False
        while not localized:
            localized = self.updateOrientationFilter() and self.getFramePosition()

    def execute(self, goal):
        print("Executing")
        self.orientation_filter = []
        self.orientation_filter_maxsize = goal.orientation_filter_size
        self.orientation_inlier_angle = goal.orientation_inlier_angle
        self.pose_filter = []
        self.pose_filter_maxsize = goal.position_filter_size
        self.pose_inlier_dist = goal.position_inlier_distance
        self.localize()
        print("Localized")
        self.server.set_succeeded()

    def loop(self, event):
        self.broadcastFrame()
        self.publishMarker()

    def updateOrientationFilter(self):
        """Uses self.poseTracker to include a new change to the whiteboard
           pose if not an outlier. If there is an update, updates frame_pose"""
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
            print("Orientation outlier found")
            self.frame_pose.orientation = Quaternion(0,0,0,1)
        if (len(self.orientation_filter) >= self.orientation_filter_maxsize):
            print("Localized orientation")
            return True
        return False

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

    def updatePoseFilter(self, position):
        """Adds position to self.pose_filter if it's not an outlier.
           Returns true if pose is stable."""
        if self.isPointOutlier(position):
            self.pose_filter = [position]
        else:
            self.pose_filter.append(position)
        if (len(self.pose_filter) >= self.pose_filter_maxsize):
            print("Localized position")
            return True
        return False

    def isPointOutlier(self, position):
        def expandPosition(p):
            return [p.x, p.y, p.z]
        pos_vec = expandPosition(position)
        for p in self.pose_filter:
            p2_vec = expandPosition(p)
            for i in range(len(pos_vec)):
                if (pos_vec[i] - p2_vec[i]) > self.pose_inlier_dist:
                    return True
        return False

    def averagePositions(self):
        def expandPosition(p):
            return [p.x, p.y, p.z]
        res_vec = [0.0, 0.0, 0.0]
        for p in self.pose_filter:
            p_vec = expandPosition(p)
            for i in range(len(res_vec)):
                res_vec[i] += p_vec[i] / len(self.pose_filter)

        return Point(res_vec[0], res_vec[1], res_vec[2])


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
        if (len(self.markerTracker.markers) == 0):
            self.frame_pose.position = self.poseTracker.pose.position
            return False
        newest = self.markerTracker.markers.values()[0]
        for tag in self.markerTracker.markers.values():
            if (tag.id in self.tags.keys()) and (tag.header.stamp.to_sec() > newest.header.stamp.to_sec()):
                newest = tag
            print(tag.id, ": ", rospy.Time.now().to_sec() - tag.header.stamp.to_sec())
        age_s = rospy.Time.now().to_sec() - newest.header.stamp.to_sec()
        if age_s > self.pose_time_threshold:
            # Too old, need a better marker
            return False

        print("Newest id:", newest.id)
        reference = self.tags[newest.id]
        ref_point = Point(reference[0], reference[1], 0.0)
        translation = computeTranslation(ref_point, self.frame_pose.orientation)
        print("Translation:", translation)
        tag_point = newest.pose.position
        
        new_position = Point(tag_point.x - translation.x, tag_point.y - translation.y, tag_point.z - translation.z)
        if self.updatePoseFilter(new_position):
            self.frame_pose.position = self.averagePositions()
            return True
        return False

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

    whiteboard_width = constants.WHITEBOARD_WIDTH #0.97
    whiteboard_height = constants.WHITEBOARD_HEIGHT #0.58
    whiteboard_depth = 0.05

    """
    corners = {16: (0.0, 0.0),
               14: (0.0, 0.5),
               0 : (-0.07, -0.06),
               1 : (0.490, -0.07),
               2 : (1.02, -0.09),
               3 : (-0.06, 0.66),
               4 : (0.50, 0.66),
               5 : (1.04, 0.63)}
    """
    corners = constants.CORNERS # {0 : (-0.1, -0.1)}
    dimensions = [whiteboard_width, whiteboard_height, whiteboard_depth]

    br = tf.TransformBroadcaster()
    tfl = tf.TransformListener()
    markers = MarkerTracker()
    sub = rospy.Subscriber('whiteboard_tags', Marker,
            callback=markers.callback) # Subscribe to AR tag poses, use reader.callback
    pose = PoseTracker()
    sub = rospy.Subscriber('plane_pose', Pose,
            callback=pose.callback) # Subscribe to whiteboard plane, use pose.callback

    pub = rospy.Publisher('whiteboard_region', Marker, queue_size=1)
    wft = WhiteboardFrameTracker(pose, markers, corners, br, pub, tfl, dimensions)

    rospy.spin()


if __name__ == '__main__':
    main()

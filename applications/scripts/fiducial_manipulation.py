#! /usr/bin/env python

import tf
import tf.transformations as tft
import numpy as np
from copy import deepcopy
import robot_api
from robot_api import Arm, Gripper
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import pickle

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String

AR_POSE_TOPIC = '/ar_pose_marker'

def relative_pose(tfl, ar_pose, pose):
    ar_mat = tfl.fromTranslationRotation([ar_pose.position.x,
                                          ar_pose.position.y,
                                          ar_pose.position.z],
                                         [ar_pose.orientation.x,
                                          ar_pose.orientation.y,
                                          ar_pose.orientation.z,
                                          ar_pose.orientation.w])
    po_mat = tfl.fromTranslationRotation([pose.position.x,
                                          pose.position.y,
                                          pose.position.z],
                                         [pose.orientation.x,
                                          pose.orientation.y,
                                          pose.orientation.z,
                                          pose.orientation.w])
    inv_mat = np.linalg.inv(ar_mat)
    res_mat = np.dot(inv_mat, po_mat)
    resTrans = tft.translation_from_matrix(res_mat)
    resRot = tft.quaternion_from_matrix(res_mat)
    res = Pose(Point(resTrans[0], resTrans[1], resTrans[2]),
               Quaternion(resRot[0], resRot[1], resRot[2], resRot[3]))
    return res


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self._markers = []
        self.markers = []

    def callback(self, msg):
        self._markers = msg.markers
        if len(self._markers) >= 2:
            self.latch()

    def latch(self):
        if len(self.markers) == 0:
            print("Found both markers")
            self.markers = deepcopy(self._markers)

    def get_id(self, tagid):
        for m in self.markers:
            if m.id == tagid:
                return m
        return None

class ProgramCreator(object):
    def __init__(self, tags, arm, gripper):
        self.state = None
        self.poses = None
        self.reader = tags
        self.arm = arm
        self.gripper = gripper
        self._tf_listener = tf.TransformListener()

    def create_program(self):
        self.state = "create"
        self.poses = []
        # Loosen the arm

    def save_pose(self, tag):
        if not self.state == "create":
            print "Not in create mode"
            return
        # Get the pose
        pose = self.arm_pose()
        ps = PoseStamped()
        ps.pose = pose
        ps.header.frame_id = "base_link"
        # Compute the pose in terms of the location of the given tag
        if not tag == "base":
            try:
                intid = int(tag)
            except Exception as e:
                print "Not an int {}".format(tag)
                return
            m = self.reader.get_id(intid)
            if m is None:
                print "Tag {} not found".format(tag)
                return
            marker_frame = m.header.frame_id
            marker_pose = m.pose.pose
            try:
                ps = self._tf_listener.transformPose(marker_frame, ps)
            except Exception as e:
                print("Couldn't convert to {}: {}".format(marker_frame, e))
                return
            ps.pose = relative_pose(self._tf_listener, marker_pose, ps.pose)
            ps.header.frame_id = tag
        # Append the pose to self.poses
        self.poses.append(ps)

    def close_gripper(self):
        if self.state != "create":
            print "Not in create mode"
            return
        # append close gripper to self.poses
        msg = String("close")
        self.poses.append(msg)

    def open_gripper(self):
        if self.state != "create":
            print "Not in create mode"
            return
        # append open gripper to self.poses
        msg = String("open")
        self.poses.append(msg)

    def save_program(self, filename):
        if self.state != "create":
            print "Not in create mode"
            return
        # Pickle self.poses to a file
        with open(filename, 'wb') as f:
            pickle.dump(self.poses, f)
        self.poses = None
        self.state = None


    def arm_pose(self):
        return self.arm.get_arm_pose()

def debug(prog):
    print "debug"
    print "Found", len(prog.reader.markers), "markers"
    if len(prog.reader.markers) > 0:
        for m in prog.reader.markers:
            print m.id
    print "curr:", prog.arm_pose()
    print prog.poses

def main():
    rospy.init_node('arm_demo')
    wait_for_time()
    
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
            callback=reader.callback) # Subscribe to AR tag poses, use reader.callback

    prog = ProgramCreator(reader, Arm(), Gripper())
    while 1:
        loop(prog)

def loop(prog):
    """Takes in commands from user:
       - Create_Program
       - Save_Pose tag
       - Open_gripper
       - Close_gripper
       - Save_Program filename"""
    command = raw_input("$: ").split()
    if len(command) < 1:
        debug(prog)
        return
    if command[0] == "create":
        prog.create_program()
        return
    if command[0] == "save_pose":
        if len(command) < 2:
            print "Invalid command"
            return
        prog.save_pose(command[1])
        return
    if command[0] == "save_program":
        if len(command) < 2:
            print "Invalid command"
            return
        prog.save_program(command[1])
        return
    if command[0] == "close_gripper":
        prog.close_gripper()
        return
    if command[0] == "open_gripper":
        prog.open_gripper()
        return
    debug(prog)
    return

if __name__ == '__main__':
    main()

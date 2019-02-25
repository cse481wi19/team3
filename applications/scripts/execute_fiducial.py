#! /usr/bin/env python

import tf
import tf.transformations as tft
import numpy as np
from copy import deepcopy
import robot_api
from robot_api import Arm, Gripper
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import pickle
import sys

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String

from visualization_msgs.msg import Marker

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
    res_mat = np.dot(ar_mat, po_mat)
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
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers

    def get_id(self, tagid):
        for m in self.markers:
            if m.id == tagid:
                return m
        return None

def makeMarker(pose):
    marker = Marker()
    marker.pose = deepcopy(pose)
    marker.type = Marker.CUBE
    marker.header.frame_id = "base_link"
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.b = 1
    marker.color.a = 1
    marker.action = Marker.ADD
    return marker

def main():
    if len(sys.argv) != 2:
        print ("Must pass name of pickled routine.")
        exit()

    rospy.init_node('arm_demo')
    wait_for_time()

    marker_publisher = rospy.Publisher('visualization_marker_new', Marker, queue_size=1)

    # Load Pickled file
    routine = []
    try:
        with open(sys.argv[1], 'rb') as f:
            routine = pickle.load(f)
    except (pickle.PickleError, IOError):
        print("Invalid pickled routine!")
        exit()

    tf_listener = tf.TransformListener()

    arm_kwargs = {'allowed_planning_time': 15, 
            'execution_timeout': 30,
            'num_planning_attempts': 5,
            'replan': True, 'group_name': 'arm'}

    # Create AR Tag reader
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback=reader.callback)

    # Create Arm Controller
    arm = Arm()

    # Create Gripper Controller
    gripper = Gripper()

    for c in routine:
        print('\n\n\n')
        print(c)
        print('\n')
        # Check if this is a gripper command.
        if hasattr(c, 'data'):
            if c.data == 'close':
                gripper.close()
            else:
                gripper.open()
            continue
        
        ps = PoseStamped()
        ps.pose = c.pose
        ps.header.frame_id = 'base_link'
        # We have a pose
        if not c.header.frame_id == 'base_link':
            marker_ps = reader.get_id(int(c.header.frame_id))
            count = 0
            while marker_ps == None:
                print('marker {} not found'.format(c.header.frame_id))
                rospy.sleep(1)
                marker_ps = reader.get_id(int(c.header.frame_id))
                count += 1
                if (count > 10):
                    print("Failed to find marker")
                    exit()
            
            ps.pose = relative_pose(tf_listener, marker_ps.pose.pose, ps.pose)

        marker = makeMarker(ps.pose)
        marker_publisher.publish(marker)
        if not (arm.move_to_pose(ps, **arm_kwargs) is None):
            print("Failed to plan")
            exit()

if __name__ == '__main__':
    main()

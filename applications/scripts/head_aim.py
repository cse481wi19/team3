#! /usr/bin/env python

import rospy
import robot_api
import tf
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def expandPoint(p):
    return [p.x, p.y, p.z]

def expandQuaternion(q):
    return [q.x, q.y, q.z, q.w]

def rotatePoint(p, quat):
    mat = tft.quaternion_matrix(expandQuaternion(quat))
    res = np.dot(mat, expandPoint(p))
    return Point(res[0], res[1], res[2])

def main():
    rospy.init_node('head_demo')
    wait_for_time()
    head = robot_api.Head()
    tfl = tf.TransformListener()

    whiteboard_pose = None
    while whiteboard_pose is None:
        try:
            (pos, quat) = tfl.lookupTransform('base_link', 'whiteboard', rospy.Time())
            whiteboard_pose = Pose(Point(pos[0], pos[1], pos[2]),
                                   Quaternion(quat[0], quat[1], quat[2], quat[3]))
        except:
            continue

    base_x = Point(1, 0, 0)
    base_y = Point(0, 1, 0)
    base_z = Point(0, 0, 1)
    whiteboard_x = rotatePoint(base_x, whiteboard_pose.orientation)
    whiteboard_y = rotatePoint(base_y, whiteboard_pose.orientation)
    whiteboard_z = rotatePoint(base_z, whiteboard_pose.orientation)

    whiteboard_z_proj_z = np.multiply(np.dot(whiteboard_z, base_z), whiteboard_z)
    whiteboard_z_xy_plane = np.subtract(whiteboard_z, whiteboard_z_proj_z)

    angle = np.arccos(np.dot(base_x, whiteboard_z_xy_plane))
    print(angle)

    #head.pan_tilt(pan, tilt)


if __name__ == '__main__':
    main()


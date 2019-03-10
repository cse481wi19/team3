#! /usr/bin/env python

import rospy
import robot_api
import tf
import tf.transformations as tft
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
import actionlib

from applications.msg import AimHeadAction

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
    p_arr = expandPoint(p)
    p_arr.append(1.0)
    res = np.dot(mat, p_arr)
    return Point(res[0], res[1], res[2])

class HeadAimer():
    def __init__(self):
        self.head = robot_api.Head()
        self.tfl = tf.TransformListener()
        """Server setup"""
        self.server = actionlib.SimpleActionServer('aim_head',
                AimHeadAction, self.execute, False)
        self.server.start()


    def execute(self, goal):
        whiteboard_pose = None
        while whiteboard_pose is None:
            try:
                (pos, quat) = self.tfl.lookupTransform('base_link', 'whiteboard', rospy.Time())
                whiteboard_pose = Pose(Point(pos[0], pos[1], pos[2]),
                                       Quaternion(quat[0], quat[1], quat[2], quat[3]))
            except:
                continue

        base_x = Point(1.0, 0.0, 0.0)
        base_y = Point(0.0, 1.0, 0.0)
        base_z = Point(0.0, 0.0, 1.0)
        whiteboard_x = rotatePoint(base_x, whiteboard_pose.orientation)
        whiteboard_y = rotatePoint(base_y, whiteboard_pose.orientation)
        whiteboard_z = rotatePoint(base_z, whiteboard_pose.orientation)
        whiteboard_z_proj = Point(whiteboard_z.x, whiteboard_z.y, 0.0)
        whiteboard_z_proj_norm = tft.vector_norm(expandPoint(whiteboard_z_proj))


        angle = np.arccos(np.dot(expandPoint(base_x), expandPoint(whiteboard_z)) / whiteboard_z_proj_norm)
        print(angle)

        self.head.pan_tilt(angle, 0.0)
        self.server.set_succeeded()


def main():
    rospy.init_node('head_aim')
    wait_for_time()
    aimer = HeadAimer()

    rospy.spin()

if __name__ == '__main__':
    main()


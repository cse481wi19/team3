#! /usr/bin/env python

import robot_api
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_python import PlanningSceneInterface
from moveit_msgs.msg import OrientationConstraint
import tf

def main():
    rospy.init_node('ee_pose_demo')

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans, rot)
        rate.sleep()

if __name__ == "__main__":
    main()

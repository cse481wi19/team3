#! /usr/bin/env python
"""An example of using waypoints to plan a cartesian path.
"""

from geometry_msgs.msg import Pose
import robot_api
import rospy
import sys


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('interpolation_demo')
    wait_for_time()

    arm = robot_api.Arm()

    poses = []

    pose1 = Pose()
    pose1.orientation.x = 0.0
    pose1.orientation.y = 0.0
    pose1.orientation.z = 0.0
    pose1.orientation.w = 1.0
    pose1.position.x = 0.0
    pose1.position.y = 0.0
    pose1.position.z = 0.0

    pose2 = Pose()
    pose2.orientation.x = 0.0
    pose2.orientation.y = 0.0
    pose2.orientation.z = 0.0
    pose2.orientation.w = 1.0
    pose2.position.x = -1.0
    pose2.position.y = 1.5
    pose2.position.z = -1.0

    pose3 = Pose()
    pose3.orientation.x = 0.0
    pose3.orientation.y = 0.0
    pose3.orientation.z = 0.0
    pose3.orientation.w = 1.0
    pose3.position.x = 1.0
    pose3.position.y = 1.5
    pose3.position.z = 1.0

    poses.append(pose1)
    poses.append(pose2)
    poses.append(pose3)

    print("Initial Poses:")
    print(poses)
    print("\n\n")

    arm.interpolate(poses)

    print("Interpolated Poses:")
    print(poses)


    """
    for i in (range(0, 10)):
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.pose.position.x = 0.65
        p.pose.position.y = 0.50 - .02 * i
        p.pose.position.z = 1.25 + .004 * i*i
        p.pose.orientation.x = 1
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        poses.append(p)
        if i == 0:
            arm.move_to_pose(p)
    """


if __name__ == '__main__':
    main()

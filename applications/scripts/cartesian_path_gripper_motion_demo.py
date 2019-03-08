#! /usr/bin/env python
"""An example of using waypoints to plan a cartesian path.
"""

from geometry_msgs.msg import PoseStamped
import robot_api
import moveit_commander
import rospy
import sys


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('straight_gripper_motion_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)

    # Set the torso height before running this demo.
    torso = robot_api.Torso()
    torso.set_height(0.4)

    arm = robot_api.Arm()

    poses = []

    pose1 = PoseStamped()
    pose1.header.frame_id = 'base_link'
    pose1.pose.position.x = 0.65
    pose1.pose.position.y = 0.6
    pose1.pose.position.z = 1.25
    pose1.pose.orientation.x = 1
    pose1.pose.orientation.y = 0
    pose1.pose.orientation.z = 0
    pose1.pose.orientation.w = 1
    poses.append(pose1)

    arm.move_to_pose(pose1)

    pose2 = PoseStamped()
    pose2.header.frame_id = 'base_link'
    pose2.pose.position.x = 0.65
    pose2.pose.position.y = 0.30
    pose2.pose.position.z = 1.25
    pose2.pose.orientation.x = 1
    pose2.pose.orientation.y = 0
    pose2.pose.orientation.z = 0
    pose2.pose.orientation.w = 1
    poses.append(pose2)

    pose3 = PoseStamped()
    pose3.header.frame_id = 'base_link'
    pose3.pose.position.x = 0.65
    pose3.pose.position.y = 0.50
    pose3.pose.position.z = 1.05
    pose3.pose.orientation.x = 1
    pose3.pose.orientation.y = 0
    pose3.pose.orientation.z = 0
    pose3.pose.orientation.w = 1
    poses.append(pose3)

    pose4 = PoseStamped()
    pose4.header.frame_id = 'base_link'
    pose4.pose.position.x = 0.65
    pose4.pose.position.y = 0.45
    pose4.pose.position.z = 1.35
    pose4.pose.orientation.x = 1
    pose4.pose.orientation.y = 0
    pose4.pose.orientation.z = 0
    pose4.pose.orientation.w = 1
    poses.append(pose4)

    pose5 = PoseStamped()
    pose5.header.frame_id = 'base_link'
    pose5.pose.position.x = 0.65
    pose5.pose.position.y = 0.40
    pose5.pose.position.z = 1.05
    pose5.pose.orientation.x = 1
    pose5.pose.orientation.y = 0
    pose5.pose.orientation.z = 0
    pose5.pose.orientation.w = 1
    poses.append(pose5)

    poses.append(pose1)

    while not rospy.is_shutdown():
        error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        rospy.sleep(0.25)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()

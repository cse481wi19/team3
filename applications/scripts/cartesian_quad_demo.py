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

    while not rospy.is_shutdown():
        error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        rospy.sleep(0.25)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()

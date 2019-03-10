#! /usr/bin/env python
"""An example of using waypoints to plan a cartesian path.
"""

from user_interface_forwarder.msg import Path
from geometry_msgs.msg import PoseStamped
import robot_api
import moveit_commander
import rospy
import sys
import tf.transformations as tft
import numpy as np

DRAW_DISTANCE = -0.05
BOARD_X_LEN = 0.16
BOARD_Y_LEN = 0.09
whiteboard_width = 0.97
whiteboard_height = 0.58

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def draw_callback(msg):
    arm = robot_api.Arm()
    path = msg.path
    path_to_execute = []
    for point in path:
        x, y = point
        pose = PoseStamped()
        pose.header.frame_id = 'whiteboard'
        pose.pose.position.x = x * BOARD_X_LEN
        pose.pose.position.y = y * BOARD_Y_LEN
        pose.pose.position.z = DRAW_DISTANCE
        print(tft.quaternion_about_axis(np.pi / 2.0, (0, 1, 0)))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        path_to_execute.append(pose)
    
    arm.move_to_pose(path_to_execute[0])
    error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
    if error is not None:
        rospy.logerr(error)
    rospy.sleep(0.25)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('straight_gripper_motion_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')
    
    path_sub = rospy.Subscriber('/user_interface_forwarder/Path', Path, draw_callback)

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)


    # Set the torso height before running this demo.
    torso = robot_api.Torso()
    torso.set_height(0.4)

    rospy.spin()


    while not rospy.is_shutdown():
        error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        rospy.sleep(0.25)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()

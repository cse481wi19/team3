#! /usr/bin/env python

import roslib
import rospy
import actionlib

from applications.msg import LocalizeWhiteboardAction, LocalizeWhiteboardGoal

if __name__ == '__main__':
    rospy.init_node('whiteboard_localize_client')
    print("Init")
    client = actionlib.SimpleActionClient('localize_whiteboard', LocalizeWhiteboardAction)
    client.wait_for_server()
    print("Created client")

    goal = LocalizeWhiteboardGoal()
    print("Created goal")
    # Fill in the goal here
    goal.orientation_filter_size = 60
    goal.position_filter_size = 60
    goal.orientation_inlier_angle = 0.2
    goal.position_inlier_distance = 0.04
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print("Done")

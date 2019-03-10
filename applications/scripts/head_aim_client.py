#! /usr/bin/env python

import roslib
import rospy
import actionlib

from applications.msg import AimHeadAction, AimHeadGoal

if __name__ == '__main__':
    rospy.init_node('head_aim_client')
    print("Init")
    client = actionlib.SimpleActionClient('aim_head', AimHeadAction)
    client.wait_for_server()
    print("Created client")

    goal = AimHeadGoal()
    print("Created goal")
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print("Done")


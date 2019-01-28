#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy

from .arm_joints import ArmJoints

ACTION_SERVER = "arm_controller/follow_joint_trajectory"

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self._client = actionlib.SimpleActionClient(ACTION_SERVER,
                control_msgs.msg.FollowJointTrajectoryAction)
        self._client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point
        

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory

        # TODO: Send goal
        # TODO: Wait for result
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.extend(arm_joints.names())
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.extend(arm_joints.values())
        point.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))
                                                                

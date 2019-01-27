#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg
import math
import rospy

ACTION_SERVER1 = 'head_controller/follow_joint_trajectory'
ACTION_SERVER2 = 'head_controller/point_head'

LOOK_AT_ACTION_NAME = 'look_at_action'  # TODO: Get the name of the look-at action
PAN_TILT_ACTION_NAME = 'pan_tilt_action'  # TODO: Get the name of the pan/tilt action
PAN_JOINT = 'head_pan_joint'  # TODO: Get the name of the head pan joint
TILT_JOINT = 'head_tilt_joint'  # TODO: Get the name of the head tilt joint
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.


class Head(object):
    """Head controls the Fetch's head.

    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians

    For example:
        head = fetch_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    MIN_PAN = -math.pi/2.0  # TODO: Minimum pan angle, in radians.
    MAX_PAN = math.pi/2.0  # TODO: Maximum pan angle, in radians.
    MIN_TILT = -math.pi/4.0  # TODO: Minimum tilt angle, in radians.
    MAX_TILT = math.pi/2.0  # TODO: Maximum tilt angle, in radians.

    def __init__(self):
        # TODO: Create actionlib clients
        self._client1 = actionlib.SimpleActionClient(
                ACTION_SERVER1, control_msgs.msg.FollowJointTrajectoryAction)
        self._client2 = actionlib.SimpleActionClient(
                ACTION_SERVER2, control_msgs.msg.PointHeadAction)
        # TODO: Wait for both servers
        self._client1.wait_for_server()#rospy.Duration(10))
        self._client2.wait_for_server()#rospy.Duration(10))

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.

        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        """
        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Fill out the goal (we recommend setting min_duration to 1 second)
        goal.trajectory.joint_names.append(TILT_JOINT)
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(x)
        point.positions.append(y)
        point.positions.append(z)
        point.time_from_start = rospy.Duration(1)
        goal.trajectory.points.append(point)
        # TODO: Send the goal
        goal.trajectory.header.frame_id = frame_id
        print(goal)
        self._client2.send_goal(goal)
        # TODO: Wait for result
        self._client2.wait_for_result()#rospy.Duration(10))
        """
        goal = control_msgs.msg.PointHeadGoal()
        goal.target.header.frame_id = frame_id
        goal.target.header.stamp = rospy.Time.now()
        goal.target.point = geometry_msgs.msg.Point(x, y, z)
        self._client2.send_goal(goal)
        self._client2.wait_for_result()


    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.

              Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # TODO: Check that the pan/tilt angles are within joint limits
        pan = min(pan, self.MAX_PAN)
        pan = max(pan, self.MIN_PAN)
        tilt = min(tilt, self.MAX_TILT)
        tilt = max(tilt, self.MIN_TILT)
        # TODO: Create a trajectory point

        # TODO: Set positions of the two joints in the trajectory point
        # TODO: Set time of the trajectory point

        # TODO: Create goal
        # TODO: Add joint names to the list
        # TODO: Add trajectory point created above to trajectory

        # TODO: Send the goal
        # TODO: Wait for result
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(pan)
        point.positions.append(tilt)
        point.time_from_start = rospy.Duration(1)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now()
        goal.trajectory.joint_names.append(PAN_JOINT)
        goal.trajectory.joint_names.append(TILT_JOINT)
        goal.trajectory.points.append(point)

        self._client1.send_goal(goal)
        self._client1.wait_for_result()

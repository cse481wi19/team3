#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy

# TODO: ACTION_NAME = ???
# TODO: JOINT_NAME = ???
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        pass

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # Clip the height to our desired range.
        height = min(height, self.MAX_HEIGHT)
        height = max(height, self.MIN_HEIGHT)

        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names.append(TORSO_JOINT_NAME)
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions.append(height)
        point.time_from_start = rospy.Duration(TIME_FROM_START)
        goal.trajectory.points.append(point)
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(10))

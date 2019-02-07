#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction

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

        # Goal builder for the arm
        self.goal_builder = MoveItGoalBuilder()

        self._moveit_goal_client = actionlib.SimpleActionClient("move_group",
                MoveGroupAction)
        self._moveit_goal_client.wait_for_server()

    def move_to_pose(self, pose_stamped):
        """Moves the end-effector to a pose, using motion planning.

        Args:
            pose: geometry_msgs/PoseStamped. The goal pose for the gripper.

        Returns:
            string describing the error if an error occurred, else None.
        """
        self.goal_builder.set_pose_goal(pose_stamped)
        goal = self.goal_builder.build()

        self._moveit_goal_client.send_goal(goal)
        self._moveit_goal_client.wait_for_result(rospy.Duration(10))
        res = self._moveit_goal_client.get_result().error_code.val
        if res == MoveItErrorCodes.SUCCESS:
            return None

        return self.moveit_error_string(res)

    def cancel_all_goals(self):
        self._client.cancel_all_goals() # Your action client from Lab 7
        self._moveit_goal_client.cancel_all_goals() # From this lab 

    def moveit_error_string(self, val):
        """Returns a string associated with a MoveItErrorCode.

        Args:
            val: The val field from moveit_msgs/MoveItErrorCodes.msg

        Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
            if the value is invalid.
        """ 
        if val == MoveItErrorCodes.SUCCESS:
            return 'SUCCESS'
        elif val == MoveItErrorCodes.FAILURE:
            return 'FAILURE'
        elif val == MoveItErrorCodes.PLANNING_FAILED:
            return 'PLANNING_FAILED'
        elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
            return 'INVALID_MOTION_PLAN'
        elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
        elif val == MoveItErrorCodes.CONTROL_FAILED:
            return 'CONTROL_FAILED'
        elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
            return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
        elif val == MoveItErrorCodes.TIMED_OUT:
            return 'TIMED_OUT'
        elif val == MoveItErrorCodes.PREEMPTED:
            return 'PREEMPTED'
        elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
            return 'START_STATE_IN_COLLISION'
        elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
            return 'GOAL_IN_COLLISION'
        elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
            return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
        elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
            return 'GOAL_CONSTRAINTS_VIOLATED'
        elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
            return 'INVALID_GROUP_NAME'
        elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
            return 'INVALID_GOAL_CONSTRAINTS'
        elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
            return 'INVALID_ROBOT_STATE'
        elif val == MoveItErrorCodes.INVALID_LINK_NAME:
            return 'INVALID_LINK_NAME'                                      
        elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
            return 'INVALID_OBJECT_NAME'
        elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
            return 'FRAME_TRANSFORM_FAILURE'
        elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
            return 'COLLISION_CHECKING_UNAVAILABLE'
        elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
            return 'ROBOT_STATE_STALE'
        elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
            return 'SENSOR_INFO_STALE'
        elif val == MoveItErrorCodes.NO_IK_SOLUTION:
            return 'NO_IK_SOLUTION'
        else:
            return 'UNKNOWN_ERROR_CODE' 

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # TODO: Create a trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        # TODO: Set position of trajectory point
        point.positions.extend(arm_joints.values())
        # TODO: Set time of trajectory point
        point.time_from_start = rospy.Duration(5)


        # TODO: Create goal
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        # TODO: Add joint name to list
        goal.trajectory.joint_names.extend(arm_joints.names())
        # TODO: Add the trajectory point created above to trajectory
        goal.trajectory.points.append(point)

        # TODO: Send goal
        self._client.send_goal(goal)
        # TODO: Wait for result
        self._client.wait_for_result(rospy.Duration(10))


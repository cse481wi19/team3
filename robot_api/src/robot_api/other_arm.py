#!/usr/bin/env python

import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy
import tf
import tf.transformations as tft

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction, OrientationConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from robot_controllers_msgs.msg import QueryControllerStatesAction, QueryControllerStatesGoal, ControllerState

ACTION_SERVER = "arm_controller/follow_joint_trajectory"
QUERY_SERVER = "query_controller_states"

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        self._client = actionlib.SimpleActionClient(ACTION_SERVER,
                control_msgs.msg.FollowJointTrajectoryAction)
        self.query_client = actionlib.SimpleActionClient(QUERY_SERVER,
                QueryControllerStatesAction)
        self._client.wait_for_server()

        self._moveit_goal_client = actionlib.SimpleActionClient("move_group",
                MoveGroupAction)
        self._moveit_goal_client.wait_for_server()
        self._compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
        self.listener = tf.TransformListener()

    def compute_ik(self, pose_stamped, timeout=rospy.Duration(5)):
        """Computes inverse kinematics for the given pose.

        Note: if you are interested in returning the IK solutions, we have
            shown how to access them.

        Args:
            pose_stamped: geometry_msgs/PoseStamped.
            timeout: rospy.Duration. How long to wait before giving up on the
                IK solution.

        Returns: True if the inverse kinematics were found, False otherwise.
        """
        request = GetPositionIKRequest()
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.group_name = 'arm'
        request.ik_request.timeout = timeout
        response = self._compute_ik(request)
        error_str = self.moveit_error_string(response.error_code.val)
        success = error_str == 'SUCCESS'
        if not success:
            return False
        joint_state = response.solution.joint_state
        for name, position in zip(joint_state.name, joint_state.position):
            if name in ArmJoints.names():
                rospy.loginfo('{}: {}'.format(name, position))
        return True    

    def check_pose(self, 
               pose_stamped,
               allowed_planning_time=10.0,
               group_name='arm',
               tolerance=0.01):
        return self.move_to_pose(
            pose_stamped,
            allowed_planning_time=allowed_planning_time,
            group_name=group_name,
            tolerance=tolerance,
            plan_only=True)

    def move_to_pose(self,
                 pose_stamped,
                 allowed_planning_time=10.0,
                 execution_timeout=15.0,
                 group_name='arm',
                 num_planning_attempts=1,
                 plan_only=False,
                 replan=False,
                 replan_attempts=5,
                 tolerance=0.01,
                 orientation_constraint=None):
        """Moves the end-effector to a pose, using motion planning.

    Args:
        pose: geometry_msgs/PoseStamped. The goal pose for the gripper.
        allowed_planning_time: float. The maximum duration to wait for a
            planning result, in seconds.
        execution_timeout: float. The maximum duration to wait for
            an arm motion to execute (or for planning to fail completely),
            in seconds.
        group_name: string. Either 'arm' or 'arm_with_torso'.
        num_planning_attempts: int. The number of times to compute the same
            plan. The shortest path is ultimately used. For random
            planners, this can help get shorter, less weird paths.
        plan_only: bool. If True, then this method does not execute the
            plan on the robot. Useful for determining whether this is
            likely to succeed.
        replan: bool. If True, then if an execution fails (while the arm is
            moving), then come up with a new plan and execute it.
        replan_attempts: int. How many times to replan if the execution
            fails.
        tolerance: float. The goal tolerance, in meters.

    Returns:
        string describing the error if an error occurred, else None.
    """
        goal_builder = MoveItGoalBuilder()
        goal_builder.set_pose_goal(pose_stamped)
        goal_builder.allowed_planning_time = allowed_planning_time
        goal_builder.num_planning_attempts = num_planning_attempts
        goal_builder.plan_only = plan_only
        goal_builder.replan = replan
        goal_builder.replan_attempts = replan_attempts
        goal_builder.tolerance = tolerance

        if orientation_constraint is not None:
            goal_builder.add_path_orientation_constraint(orientation_constraint)

        goal = goal_builder.build()

        self._moveit_goal_client.send_goal(goal)
        self._moveit_goal_client.wait_for_result(rospy.Duration(execution_timeout))
        res = self._moveit_goal_client.get_result()
        if res is None:
            return None
        if res.error_code.val == MoveItErrorCodes.SUCCESS:
            return None

        return self.moveit_error_string(res.error_code.val)

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

    def straight_move_to_pose(self,
                              group,
                              pose_stamped,
                              ee_step=0.025,
                              jump_threshold=2.0,
                              avoid_collisions=True):
        """Moves the end-effector to a pose in a straight line.
        Args:
          group: moveit_commander.MoveGroupCommander. The planning group for
            the arm.
          pose_stamped: geometry_msgs/PoseStamped. The goal pose for the
            gripper.
          ee_step: float. The distance in meters to interpolate the path.
          jump_threshold: float. The maximum allowable distance in the arm's
            configuration space allowed between two poses in the path. Used to
            prevent "jumps" in the IK solution.
          avoid_collisions: bool. Whether to check for obstacles or not.
        Returns:
            string describing the error if an error occurred, else None.
        """
        # Transform pose into planning frame
        self._tf_listener.waitForTransform(pose_stamped.header.frame_id,
                                           group.get_planning_frame(),
                                           rospy.Time.now(),
                                           rospy.Duration(1.0))
        try:
            pose_transformed = self._tf_listener.transformPose(
                group.get_planning_frame(), pose_stamped)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr('Unable to transform pose from frame {} to {}'.format(
                pose_stamped.header.frame_id, group.get_planning_frame()))
            return moveit_error_string(
                MoveItErrorCodes.FRAME_TRANSFORM_FAILURE)

        # Compute path
        plan, fraction = group.compute_cartesian_path(
            [group.get_current_pose().pose,
             pose_transformed.pose], ee_step, jump_threshold, avoid_collisions)
        if fraction < 1 and fraction > 0:
            rospy.logerr(
                'Only able to compute {}% of the path'.format(fraction * 100))
        if fraction == 0:
            return moveit_error_string(MoveItErrorCodes.PLANNING_FAILED)

        # Execute path
        result = group.execute(plan, wait=True)
        if not result:
            return moveit_error_string(MoveItErrorCodes.INVALID_MOTION_PLAN)
        else:
            return None


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
    
    def get_arm_pose(self):
        try:
            (trans, rot) = self.listener.lookupTransform('/base_link',
                    '/wrist_roll_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None
        res = Pose(Point(*trans), Quaternion(*rot))
        return res

    def relax_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.STOPPED
        goal.updates.append(state)
        self.query_client.send_goal(goal)
        self.query_client.wait_for_result()

    def enable_arm(self):
        goal = QueryControllerStatesGoal()
        state = ControllerState()
        state.name = 'arm_controller/follow_joint_trajectory'
        state.state = ControllerState.RUNNING
        goal.updates.append(state)
        self.query_client.send_goal(goal)
        self.query_client.wait_for_result()

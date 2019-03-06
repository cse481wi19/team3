#!/usr/bin/env python

import actionlib
import control_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import math
import moveit_commander
import rospy
import tf

from .arm_joints import ArmJoints
from .moveit_goal_builder import MoveItGoalBuilder
from moveit_msgs.msg import MoveItErrorCodes, MoveGroupAction
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from tf.listener import TransformListener

def point_to_point_segmenter(pose1, pose2, granularity):



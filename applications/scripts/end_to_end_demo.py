#! /usr/bin/env python
"""An example of using waypoints to plan a cartesian path.
"""

from user_interface_forwarder.msg import Path
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import robot_api
import moveit_commander
import rospy
import sys
import tf.transformations as tft
import tf
import numpy as np
import robot_api.constants as constants
from visualization_msgs.msg import Marker
from copy import deepcopy

DRAW_DISTANCE = -0.29
BOARD_X_LEN = constants.WHITEBOARD_WIDTH #0.64
BOARD_Y_LEN = constants.WHITEBOARD_HEIGHT #0.36
BOARD_X_OFFSET = 0.05
BOARD_Y_OFFSET = 0.1
BOARD_Z_OFFSET = constants.WHITEBOARD_Z_OFFSET #-0.3

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def makeMarker(mid, pose):
    marker = Marker()
    marker.id = mid
    marker.type = Marker.ARROW
    marker.scale.x = 0.1
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose = deepcopy(pose.pose)
    """Shift whiteboard marker position so top corner is correctly placed"""
    marker.header.frame_id = 'whiteboard'
    return marker

class DrawClass():
    def __init__(self, pub, tfl, group, restpose):
        self.pub = pub
        self.tfl = tfl
        self.group = group
        self.restpose = restpose

    def draw_callback(self, msg):
        arm = robot_api.Arm()
        paths = msg.path
        path_to_execute = []
        #path = paths[0]
        mid = 1
        point = paths[0]
        x = point.x
        y = point.y
        pose = PoseStamped()
        pose.header.frame_id = 'whiteboard'
        pose.pose.position.x = x * BOARD_X_LEN + BOARD_X_OFFSET
        pose.pose.position.y = y * BOARD_Y_LEN + BOARD_Y_OFFSET
        pose.pose.position.z = DRAW_DISTANCE + BOARD_Z_OFFSET
        quat = tft.quaternion_about_axis(-np.pi / 2.0, (0, 1, 0))
        self.restpose.pose.orientation.x = quat[0]
        self.restpose.pose.orientation.y = quat[1]
        self.restpose.pose.orientation.z = quat[2]
        self.restpose.pose.orientation.w = quat[3]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        try:
            pose = self.tfl.transformPose('base_link', pose)
        except:
            print("Couldn't convert point firstpose")
        path_to_execute.append(pose)

        for point in paths:
            x = point.x
            y = point.y
            pose = PoseStamped()
            pose.header.frame_id = 'whiteboard'
            pose.pose.position.x = x * BOARD_X_LEN + BOARD_X_OFFSET
            pose.pose.position.y = y * BOARD_Y_LEN + BOARD_Y_OFFSET
            pose.pose.position.z = DRAW_DISTANCE
            quat = tft.quaternion_about_axis(-np.pi / 2.0, (0, 1, 0))
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]


            m = makeMarker(mid, pose)
            self.pub.publish(m)
            mid += 1

            try:
                pose = self.tfl.transformPose('base_link', pose)
            except:
                print("Couldn't convert point pose")

            path_to_execute.append(pose)

        print("Completed path with ", len(path_to_execute), " points")

        try:
            firstpose = self.tfl.transformPose('base_link', self.restpose)
        except:
            print("Couldn't convert firstpose")
            return

        arm.move_to_pose(firstpose)
        print("Moved to firstpose")
        arm.move_to_pose(path_to_execute[0])
        print("Moved to path_to_execute[0]")
        error = arm.cartesian_path_move(self.group, path_to_execute, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        print("Completed cartesian_path_move")
        arm.move_to_pose(firstpose)
        rospy.sleep(0.25)

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('end_to_end_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    # Set the torso height before running this demo.
    torso = robot_api.Torso()
    torso.set_height(0.4)

    RestPose = PoseStamped()
    RestPose.header.frame_id = 'whiteboard'
    RestPose.pose.position = Point(0.4, 0.4, -0.65)

    tfl = tf.TransformListener()
    pub = rospy.Publisher('drawing_points', Marker, queue_size=100)
    drawer = DrawClass(pub, tfl, group, RestPose)
    
    path_sub = rospy.Subscriber('/user_interface_forwarder/Path', Path, drawer.draw_callback)

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)

    rospy.spin()

if __name__ == '__main__':
    main()

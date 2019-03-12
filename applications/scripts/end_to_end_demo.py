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

from applications.msg import WhiteboardObstaclesGoal, WhiteboardObstaclesAction

PREDRAW_Z_OFFSET = constants.PREDRAW_Z_OFFSET
BOARD_X_LEN = constants.WHITEBOARD_WIDTH #0.64
BOARD_Y_LEN = constants.WHITEBOARD_HEIGHT #0.36
BOARD_X_OFFSET = 0.0
BOARD_Y_OFFSET = 0.0
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
    def __init__(self, pub, tfl, group, restpose, arm, client):
        self.pub = pub
        self.tfl = tfl
        self.group = group
        self.restpose = restpose
        self.arm = arm
        self.client = client

    def draw_callback(self, msg):
        paths = msg.path
        path_to_execute = []
        #path = paths[0]
        mid = 1

        # Add first postdraw point
        point = paths[0]
        x = point.x
        y = point.y
        pose = PoseStamped()
        pose.header.frame_id = 'whiteboard'
        pose.pose.position.x = x * BOARD_X_LEN + BOARD_X_OFFSET
        pose.pose.position.y = y * BOARD_Y_LEN + BOARD_Y_OFFSET
        pose.pose.position.z = PREDRAW_Z_OFFSET + BOARD_Z_OFFSET
        quat = tft.quaternion_about_axis(-np.pi / 2.0, (0, 1, 0))
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
            pose.pose.position.z = BOARD_Z_OFFSET
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

        # Add last postdraw point
        point = paths[-1]
        x = point.x
        y = point.y
        pose = PoseStamped()
        pose.header.frame_id = 'whiteboard'
        pose.pose.position.x = x * BOARD_X_LEN + BOARD_X_OFFSET
        pose.pose.position.y = y * BOARD_Y_LEN + BOARD_Y_OFFSET
        pose.pose.position.z = PREDRAW_Z_OFFSET + BOARD_Z_OFFSET
        quat = tft.quaternion_about_axis(-np.pi / 2.0, (0, 1, 0))
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        try:
            pose = self.tfl.transformPose('base_link', pose)
        except:
            print("Couldn't convert point firstpose")
        path_to_execute.append(pose)


        print("Completed path with ", len(path_to_execute), " points")

        try:
            firstpose = self.tfl.transformPose('base_link', self.restpose)
        except:
            print("Couldn't convert firstpose")
            return
        
        self.addAllScenes()
        self.arm.move_to_pose(firstpose)
        print("Moved to firstpose")
        self.arm.move_to_pose(path_to_execute[0])
        rospy.sleep(0.25)
        self.removeAllScenes()

        print("Moved to path_to_execute[0]")
        error = self.arm.cartesian_path_move(self.group, path_to_execute, jump_threshold=2.0)
        rospy.sleep(0.25)

        self.addAllScenes()
        if error is not None:
            rospy.logerr(error)
        print("Completed cartesian_path_move")
        self.arm.move_to_pose(firstpose)
        rospy.sleep(0.25)

    def addAllScenes(self):
        goal = WhiteboardObstaclesGoal()
        goal.obstacle = "tray"
        goal.add = True
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        goal.obstacle = "wall"
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        goal.obstacle = "block"
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

    def removeAllScenes(self):
        goal = WhiteboardObstaclesGoal()
        goal.obstacle = "tray"
        goal.add = False
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        goal.obstacle = "wall"
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))
        goal.obstacle = "block"
        self.client.send_goal(deepcopy(goal))
        self.client.wait_for_result(rospy.Duration.from_sec(5.0))

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('end_to_end_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')
    arm = robot_api.Arm()

    # Set the torso height before running this demo.
    torso = robot_api.Torso()
    torso.set_height(0.4)

    RestPose = PoseStamped()
    RestPose.header.frame_id = 'base_link'
    RestPose.pose.position = Point(0.45, -0.5, 1.25)
    RestPose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    path_sub = rospy.Subscriber('/user_interface_forwarder/Path', Path, drawer.draw_callback)

    client = actionlib.SimpleActionClient('set_whiteboard_obstacles', WhiteboardObstaclesAction)

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)

    #arm.move_to_pose(RestPose)

    tfl = tf.TransformListener()
    pub = rospy.Publisher('drawing_points', Marker, queue_size=100)
    drawer = DrawClass(pub, tfl, group, RestPose, arm, client)

    rospy.spin()

if __name__ == '__main__':
    main()

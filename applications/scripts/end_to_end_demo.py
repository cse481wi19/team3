#! /usr/bin/env python
"""An example of using waypoints to plan a cartesian path.
"""

from user_interface_forwarder.msg import Path
from geometry_msgs.msg import PoseStamped
import robot_api
import moveit_commander
import rospy
import sys
import tf.transformations as tft
import numpy as np
from visualization_msgs.msg import Marker

DRAW_DISTANCE = -0.05
BOARD_X_LEN = 0.16
BOARD_Y_LEN = 0.09
BOARD_X_OFFSET = 0.3
BOARD_Y_OFFSET = 0.3
whiteboard_width = 0.97
whiteboard_height = 0.58

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def makeMarker(mid, pose):
    marker = Marker()
    marker.id = mid
    marker.type = Marker.SPHERE
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.pose = Pose()
    """Shift whiteboard marker position so top corner is correctly placed"""
    marker.pose.position = deepcopy(pose)
    marker.pose.orientation = Quaternion(0, 0, 0, 1)
    marker.header.frame_id = 'whiteboard'
    return marker

class DrawClass():
    def __init__(self, pub):
        self.pub = pub

    def draw_callback(self, msg):
        arm = robot_api.Arm()
        paths = msg.path
        path_to_execute = []
        #path = paths[0]
        mid = 1
        for point in paths:
            x = point.x
            y = point.y
            pose = PoseStamped()
            pose.header.frame_id = 'whiteboard'
            pose.pose.position.x = x * BOARD_X_LEN + BOARD_X_OFFSET
            pose.pose.position.y = y * BOARD_Y_LEN + BOARD_Y_OFFSET
            pose.pose.position.z = DRAW_DISTANCE
            quat = tft.quaternion_about_axis(np.pi / 2.0, (0, 1, 0))
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            m = makeMarker(mid, pose)
            self.pub.publish(m)
            mid += 1

            path_to_execute.append(pose)

        print(path_to_execute)

        '''
        arm.move_to_pose(path_to_execute[0])
        error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        rospy.sleep(0.25)
        '''

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('straight_gripper_motion_demo')
    wait_for_time()
    moveit_robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    pub = rospy.Publisher('drawing_points', Marker, queue_size=100)
    drawer = DrawClass(pub)
    
    path_sub = rospy.Subscriber('/user_interface_forwarder/Path', Path, drawer.draw_callback)

    def on_shutdown():
        group.stop()
        moveit_commander.roscpp_shutdown()

    rospy.on_shutdown(on_shutdown)


    # Set the torso height before running this demo.
    torso = robot_api.Torso()
    torso.set_height(0.4)

    rospy.spin()


    while not rospy.is_shutdown():
        error = arm.cartesian_path_move(group, poses, jump_threshold=2.0)
        if error is not None:
            rospy.logerr(error)
        rospy.sleep(0.25)

    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()

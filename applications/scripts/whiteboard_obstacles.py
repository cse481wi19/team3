#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import robot_api
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def print_usage():
    print 'Usage: rosrun applications a5_obstacles.py'
    print 'Drive the robot until the PlanningScene lines up with the point cloud.'


def main():
    rospy.init_node('whiteboard_obstacles')
    wait_for_time()

    tfl = tf.TransformListener()

    planning_scene = PlanningSceneInterface('base_link')
    planning_scene.clear()
    planning_scene.removeCollisionObject('floor')
    box_pose = PoseStamped()
    box_pose.header.frame_id = "whiteboard"
    box_pose.pose.orientation.w = 1.0

    res_pose = None
    while res_pose is None:
        try:
            res_pose = tfl.transformPose('base_link', box_pose)
        except:
            rospy.sleep(1)
    p = SolidPrimitive()
    p.type = SolidPrimitive.BOX
    p.dimensions = [20.0, 3.0, 0.1]
    planning_scene.addSolidPrimitive('floor', p, res_pose.pose)

    rospy.sleep(2)


if __name__ == '__main__':
    main()


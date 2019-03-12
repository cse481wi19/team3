#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
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
    planning_scene.removeCollisionObject('wall')
    planning_scene.removeCollisionObject('tray')
    planning_scene.removeAttachedObject('block')

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
    p.dimensions = [10.0, 3.0, 0.1]
    planning_scene.addSolidPrimitive('wall', p, res_pose.pose)
    tray = SolidPrimitive()
    tray.type = SolidPrimitive.BOX
    tray.dimensions = [10.0, 0.85, 0.2]
    res_pose.pose.position.z = tray.dimensions[1] / 2.0
    res_pose.pose.position.x -= tray.dimensions[2] / 2.0
    planning_scene.addSolidPrimitive('tray', tray, res_pose.pose)

    frame_attached_to = 'gripper_link'
    frames_okay_to_collide_with = [
        'gripper_link', 'l_gripper_finger_link', 'r_gripper_finger_link'        
    ]
    planning_scene.attachBox('block', 0.13, 0.05, 0.05, 0.0, 0.0, 0,
            frame_attached_to, frames_okay_to_collide_with)

    rospy.sleep(1)


if __name__ == '__main__':
    main()


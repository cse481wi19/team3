#! /usr/bin/env python

from moveit_python import PlanningSceneInterface
import robot_api
import rospy


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

    planning_scene = PlanningSceneInterface('whiteboard')
    planning_scene.clear()
    planning_scene.removeCollisionObject('table')
    #planning_scene.removeCollisionObject('floor')
    planning_scene.addBox('floor', 5, 5, 0.01, 0, 0, 0.01/2)
    #planning_scene.addBox('table', 1, 1, 0.02, 1.25, 0, 0.75)

    rospy.sleep(2)


if __name__ == '__main__':
    main()


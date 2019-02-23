#! /usr/bin/env python

from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, Quaternion
import robot_api
import rospy
from copy import deepcopy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ArTagReader(object):
    def __init__(self):
        self.markers = []

    def callback(self, msg):
        self.markers = msg.markers


def main():
    rospy.init_node('WillSmithGenie')
    wait_for_time()

    start = PoseStamped()
    start.header.frame_id = 'base_link'
    start.pose.position.x = 0.5
    start.pose.position.y = 0.5
    start.pose.position.z = 0.75
    arm = robot_api.Arm()
    arm.move_to_pose(start)
    arm_kwargs = {
        'allowed_planning_time': 15,
        'execution_timeout': 30,
        'num_planning_attempts': 5,
        'replan': False,
        'group_name': 'arm_with_torso'
    }
                                                                               
    reader = ArTagReader()
    sub = rospy.Subscriber('/ar_pose_marker', AlvarMarkers,
            callback=reader.callback) # Subscribe to AR tag poses, use reader.callback
    
    while len(reader.markers) == 0:
        rospy.sleep(0.1)
    
    for marker in reader.markers:
        # TODO: get the pose to move to
        ps = deepcopy(marker.pose)
        ps.header.frame_id = 'base_link'
        ps.pose.orientation = Quaternion(0, 0, 0, 1)
        error = arm.move_to_pose(ps, **arm_kwargs)
        if error is None:
            rospy.loginfo('Moved to marker {}'.format(marker.id))
            return
        else:
            rospy.logwarn('Failed to move to marker {}'.format(marker.id))
    rospy.logerr('Failed to move to any markers!')


if __name__ == '__main__':
    main()

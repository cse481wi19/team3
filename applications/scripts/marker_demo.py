#!/usr/bin/env python

import rospy
from copy import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry
import math

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def distance(pose1, pose2):
    return math.sqrt((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2 + (pose1.position.z - pose2.position.z)**2)

class NavPath(object):

    def __init__(self, marker_publisher):
        self._path = []
        self._marker_publisher = marker_publisher
        self._lastpose = None

    def callback(self, msg):
        rospy.loginfo(msg)
        if not self._lastpose is None:
            print(distance(self._lastpose, msg.pose.pose))
        if self._lastpose is None:
            self._path.append(copy(msg.pose.pose))
            self.plot_points()
            self._lastpose = msg.pose.pose
        elif distance(self._lastpose, msg.pose.pose) > .2:
            print(distance(self._lastpose, msg.pose.pose))
            self._path.append(copy(msg.pose.pose))
            self.plot_points()
            self._lastpose = msg.pose.pose


    def plot_points(self):
        markers = [Marker(
                    type=Marker.SPHERE,
                    id=i,
                    action=0,
                    pose=pose,
                    scale=Vector3(0.1, 0.1, 0.1),
                    header=Header(frame_id='odom'),
                    color=ColorRGBA(1.0, 1.0, 1.0, 1.0)) for i, pose in enumerate(self._path)]
        arr = MarkerArray(markers)
        self._marker_publisher.publish(arr)

def main():
  rospy.init_node('my_node')
  wait_for_time()
  marker_publisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)
  rospy.sleep(2.0)
  nav = NavPath(marker_publisher)
  odom_sub = rospy.Subscriber('odom', Odometry, callback=nav.callback)
  rospy.spin()

def show_text_in_rviz(marker_publisher, text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(2),
                pose=Pose(Point(0, 0, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.12, 0.12, 0.12),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(1.0, 0.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish([marker])

if __name__ == '__main__':
  main()

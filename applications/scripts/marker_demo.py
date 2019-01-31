#!/usr/bin/env python

import rospy
from copy import copy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
from nav_msgs.msg import Odometry

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

class NavPath(object):

    def __init__(self, marker_publisher):
        self._path = []
        self._marker_publisher = marker_publisher
        self._last = 0

    def callback(self, msg):
        rospy.loginfo(msg)
        if rospy.Time().now().to_sec() - self._last > 1:
            self._path.append(copy(msg.pose.pose))
            self.plot_points()
            self._last = rospy.Time().now().to_sec()

    def plot_points(self):
        markers = [Marker(
                    type=Marker.LINE_STRIP,
                    id=i,
                    ns="Name",
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
  rospy.sleep(0.5)
  show_text_in_rviz(marker_publisher, 'Kariboh? Your deck is bad.')
  rospy.sleep(2.0)
  show_text_in_rviz(marker_publisher, 'My grandfather\'s deck has no bad cards.')
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

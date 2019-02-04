#!/usr/bin/env python

import rospy
from map_annotator.msg import PoseNames, UserAction
from cli_annotator import Annotator
from geometry_msgs.msg import Pose, Point, Quaternion
from marker_tracker import MarkerTracker


def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

class Database():

    def __init__(self):
        self.pub = rospy.Publisher('map_annotator/pose_names',
                                    PoseNames,
                                    queue_size=1,
                                    latch=True)
        self.ann = Annotator("poses")
        self.markers = MarkerTracker(self.ann)
        self.sub = rospy.Subscriber('map_annotator/user_actions',
                                     UserAction,
                                     callback=self.callback)
        self._update_list()

    def callback(self, evt):
        command = evt.command
        if command == evt.CREATE:
            self.create(evt.name)
        elif command == evt.DELETE:
            self.delete(evt.name)
        elif command == evt.GOTO:
            self.goto(evt.name)

    def create(self, name):
        print("create")
        self.markers.create(name)
        self._update_list()

    def delete(self, name):
        print("delete")
        self.markers.delete(name)
        self._update_list()

    def goto(self, name):
        print("goto")
        self.markers.goto(name)
        self._update_list()

    def rename_pose(self):
        pass

    def _update_list(self):
        posenames = PoseNames()
        posenames.names = self.ann.list()
        self.pub.publish(posenames)

def main():
    database = Database()
    rospy.spin()


if __name__ == '__main__':
    main()

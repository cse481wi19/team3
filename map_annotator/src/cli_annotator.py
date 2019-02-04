#! /usr/bin/env python

import sys
import pickle
import robot_api
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class PicklePoseStore(object):
    def __init__(self, filename):
        self.filename = filename
        try:
            with open(self.filename, 'rb') as f:
                self.local_dict = pickle.load(f)
        except (pickle.PickleError, IOError):
            self.local_dict = {}

    def list(self):
        return self.local_dict.copy()

    def save(self, name, pose):
        self.local_dict[name] = pose
        self._write()

    def delete(self, name):
        if name in self.local_dict:
            del self.local_dict[name]
            self._write()
            return True
        return False

    def get(self, name):
        try:
            return self.local_dict[name]
        except KeyError:
            return None

    def _write(self):
        print(self.local_dict)
        with open(self.filename, 'wb') as f:
            pickle.dump(self.local_dict, f)

class Annotator(object):
    def __init__(self, filename):
        self.pose_store = PicklePoseStore(filename)
        rospy.init_node('map_annotator', anonymous=False)
        self.frame_id = 'map'
        self.goal_pub = rospy.Publisher(
                'move_base_simple/goal',
                PoseStamped,
                queue_size=10)
        self.pose_sub = rospy.Subscriber(
                'amcl_pose',
                PoseWithCovarianceStamped,
                callback=self._pose_callback,
                queue_size=10)
        self.pose = None

    def list(self):
        return self.pose_store.list()

    def save(self, name, pose=None):
        if pose is None:
            pose = self.pose
        if pose is not None:
            self.pose_store.save(name, pose)
        return pose

    def delete(self, name):
        return self.pose_store.delete(name)

    # returns whether a goal was successfully published
    def goto(self, name):
        pose = self.pose_store.get(name)
        if pose is not None:
            goal = PoseStamped()
            goal.header.frame_id = self.frame_id
            goal.header.stamp = rospy.Time.now()
            goal.pose = pose
            self.goal_pub.publish(goal)
        return pose is not None

    def _pose_callback(self, msg):
        self.pose = msg.pose.pose

def print_commands():
    print("Commands")
    print("  list: List saved poses.")
    print("  save <name>: Save the robot's current pose as <name>. Overwrites if <name> already exists.")
    print("  delete <name>: Delete the pose given by <name>.")
    print("  goto <name>: Sends the robot to the pose given by <name>.")
    print("  help: Show this list of commands")

def main():
    if len(sys.argv) < 2:
        print("need pickle filename")
        return
    filename = sys.argv[1]
    backend = Annotator(filename)
    print("Welcome to the map annotator!")
    print_commands()
    while(True):
        command = raw_input()
        if command.startswith('list'):
            print(backend.list().keys())
        elif command.startswith('save'):
            name = command[5:]
            pose = backend.save(name)
            if pose is None:
                print('Warning: no poses have been published by the localizer')
        elif command.startswith('delete'):
            name = command[7:]
            success = backend.delete(name)
            if not success:
                print(str(name) + " not found")
        elif command.startswith('goto'):
            name = command[5:]
            success = backend.goto(name)
            if not success:
                print(str(name) + " not found")
        elif command.startswith('help'):
            print_commands()
        elif command.startswith('exit'):
            return
        else:
            print("Command not found, please choose another")


if __name__ == "__main__":
    main()

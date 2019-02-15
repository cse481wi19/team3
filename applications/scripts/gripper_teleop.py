#! /usr/bin/env python

import robot_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
import tf
from copy import deepcopy
import random

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass


class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server, tf_listener):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._tf_listener = tf_listener
        self.name = "grippy"
        self.arm_kwargs = {
            'allowed_planning_time': 15,
            'execution_timeout': 30,
            'num_planning_attempts': 5,
            'replan': False,
            'group_name': 'arm'
        }

    def _markerList(self, pose):
        res = []
        def makeMarker(resource):
            marker = Marker()
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = resource
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.5
            marker.color.a = 1.0
            marker.pose = deepcopy(pose)
            return marker
        hand = makeMarker(GRIPPER_MESH)
        l = makeMarker(L_FINGER_MESH)
        l.pose.position.y = -0.05
        r = makeMarker(R_FINGER_MESH)
        r.pose.position.y = 0.05
        res.append(hand)
        res.append(l)
        res.append(r)
        return res

    def _make_6dof_controls(self):
        res = []
        def makeControl(mode, axis):
            control = InteractiveMarkerControl()
            control.interaction_mode = mode
            control.always_visible = True
            control.name = axis + str(mode)
            control.orientation.w = 1
            if axis == 'x':
                control.orientation.x = 1
                control.orientation.y = 0
                control.orientation.z = 0
            if axis == 'y':
                control.orientation.x = 0
                control.orientation.y = 0
                control.orientation.z = 1
            if axis == 'z':
                control.orientation.x = 0
                control.orientation.y = 1
                control.orientation.z = 0
            return control
        res.append(makeControl(InteractiveMarkerControl.ROTATE_AXIS, 'x'))
        res.append(makeControl(InteractiveMarkerControl.ROTATE_AXIS, 'y'))
        res.append(makeControl(InteractiveMarkerControl.ROTATE_AXIS, 'z'))
        res.append(makeControl(InteractiveMarkerControl.MOVE_AXIS, 'x'))
        res.append(makeControl(InteractiveMarkerControl.MOVE_AXIS, 'y'))
        res.append(makeControl(InteractiveMarkerControl.MOVE_AXIS, 'z'))
        return res

    def _make_menu_entries(self):
        res = []
        def makeEntry(name, i):
            m = MenuEntry()
            m.command_type = MenuEntry.FEEDBACK
            m.id = i
            m.title = name
            m.command = name
            return m
        res.append(makeEntry("Move", 1))
        res.append(makeEntry("Take", 2))
        res.append(makeEntry("Use", 3))
        res.append(makeEntry("Magic", 4))
        return res

    def start(self):
        
        # Pose that defines position and frame for IM
        gripperOffset = Pose()
        gripperOffset.position.x = 0.166
        gripperOffset.position.y = 0
        gripperOffset.position.z = 0
        ps = PoseStamped()
        ps.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        ps.header.frame_id = "wrist_roll_link"
        try:
            ps = self._tf_listener.transformPose('base_link', ps)
        except (e):
            print("Couldn't convert to base_link: {}".format(e))

        gripper_im = InteractiveMarker()
        gripper_im.header.frame_id = ps.header.frame_id
        gripper_im.name = self.name
        gripper_im.pose = ps.pose

        gripper_im.menu_entries = self._make_menu_entries()

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.MENU
        button_control.name = "MENU"
        button_control.always_visible = True
        button_control.markers += self._markerList(gripperOffset)
        gripper_im.controls.append(button_control)

        gripper_im.controls.extend(self._make_6dof_controls())
        gripper_im.scale = 0.5
        gripper_im.scale = 0.5
        gripper_im.scale = 0.5

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _getGripperMarkers(self, gripper_im):
        for button in gripper_im.controls:
            if button.name == "MENU":
                return button.markers
        return None

    def _setGripperOpen(self):
        gripper_im = self._im_server.get(self.name)
        for m in self._getGripperMarkers(gripper_im):
            if m.mesh_resource == L_FINGER_MESH:
                m.pose.position.y = -0.05
                print("Open:", m.pose.position.y)
            if m.mesh_resource == R_FINGER_MESH:
                m.pose.position.y = 0.05

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _setGripperClosed(self):
        gripper_im = self._im_server.get(self.name)
        for m in self._getGripperMarkers(gripper_im):
            if m.mesh_resource == L_FINGER_MESH:
                m.pose.position.y = -0.1
                print("Closed:", m.pose.position.y)
            if m.mesh_resource == R_FINGER_MESH:
                m.pose.position.y = 0.1
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _resetGripperPosition(self):
        gripper_im = self._im_server.get(self.name)
        # Pose that defines position and frame for IM

        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()


    def _setGripperColor(self, r, g, b):
        gripper_im = self._im_server.get(self.name)
        for m in self._getGripperMarkers(gripper_im):
            m.color.r = r
            m.color.g = g
            m.color.b = b
        self._im_server.insert(gripper_im, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def _poseUpdate(self, feedback):
        print("poseUpdate")
        pose = feedback.pose
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose = pose
        if self._arm.compute_ik(ps):
            rospy.loginfo('Found IK!')
            self._setGripperColor(0.0, 0.5, 0.5)
        else:
            rospy.loginfo('No IK found.')
            self._setGripperColor(1.0, 0.0, 0.0)

    def _menuMove(self, feedback):
        pose = deepcopy(feedback.pose)
        ps = PoseStamped()
        ps.header.frame_id = 'base_link'
        ps.pose = pose
        error = self._arm.move_to_pose(ps, **self.arm_kwargs)
        if error is not None:
            rospy.logerr('menuMove failed: {}'.format(error))
            self._setGripperColor(1.0, 0.0, 0.0)
        else:
            rospy.loginfo('menuMove succeeded')
            self._resetGripperPosition()
            self._setGripperColor(0.0, 1.0, 0.0)


    def _menuTake(self, feedback):
        self._setGripperClosed()
        self._gripper.close()

    def _menuUse(self, feedback):
        self._setGripperOpen()
        self._gripper.open()

    def _menuMagic(self, feedback):
        print("Oooh, magic!")
        self._setGripperColor(random.random(), random.random(), random.random())

    def _menuSelect(self, feedback):
        if feedback.menu_entry_id == 1:
            self._menuMove(feedback)
        if feedback.menu_entry_id == 2:
            self._menuTake(feedback)
        if feedback.menu_entry_id == 3:
            self._menuUse(feedback)
        if feedback.menu_entry_id == 4:
            self._menuMagic(feedback)

    def handle_feedback(self, feedback):
        event_type = feedback.event_type
        if event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self._poseUpdate(feedback)
        elif event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self._menuSelect(feedback)

class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server

    def start(self):
        # obj_im = InteractiveMarker() ...
        #self._im_server.insert(obj_im, feedback_cb=self.handle_feedback)
        pass

    def handle_feedback(self, feedback):
        pass


def main():
    rospy.init_node("gripper_teleop")
    wait_for_time()
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    im_server = InteractiveMarkerServer('gripper_im_server')
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server')
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    teleop = GripperTeleop(arm, gripper, im_server, listener)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()

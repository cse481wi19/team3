#! /usr/bin/env python

import robot_api
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, MenuEntry
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3, PoseStamped
import tf
import tf.transformations as tft
from copy import deepcopy
import random
import numpy as np

GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

def wait_for_time():                                              
    """Wait for simulated time to begin.                          
    """                                                           
    while rospy.Time().now().to_sec() == 0:                       
        pass

def make_6dof_controls():
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


def gripperMarkerList(pose):
    """Returns a list of markers representing the gripper, with given pose"""
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
    l.pose.position.y += -0.05
    r = makeMarker(R_FINGER_MESH)
    r.pose.position.y += 0.05
    res.append(hand)
    res.append(l)
    res.append(r)
    return res

def make_menu_entries(entries):
    res = []
    def makeEntry(name, i):
        m = MenuEntry()
        m.command_type = MenuEntry.FEEDBACK
        m.id = i
        m.title = name
        m.command = name
        return m
    for i, e in enumerate(entries):
        res.append(makeEntry(e, i + 1))
    return res

def matrixFromPose(pose):
    transM = tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
    rotM = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    return np.add(np.add(transM, rotM), -1.0 * np.identity(4))

def translatePose(pose, point):
    """Translates a pose (preserving orientation) by the given x, y, z input"""
    transPose = Pose(point, Quaternion())
    tM = matrixFromPose(transPose)
    pM = matrixFromPose(pose)
    rM = np.dot(pM, tM)
    resTrans = tft.translation_from_matrix(rM)
    resRot = tft.quaternion_from_matrix(rM)
    res = Pose(Point(resTrans[0], resTrans[1], resTrans[2]),
               Quaternion(resRot[0], resRot[1], resRot[2], resRot[3]))
    return res

class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server, tf_listener):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._tf_listener = tf_listener
        self.name = "grippy"
        self.arm_kwargs = {'allowed_planning_time': 15, 
                'execution_timeout': 30,
                'num_planning_attempts': 5,
                'replan': False, 'group_name': 'arm'}

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

        gripper_im.menu_entries = make_menu_entries(["Move", "Take", "Use", "Magic"])

        button_control = InteractiveMarkerControl()
        button_control.interaction_mode = InteractiveMarkerControl.MENU
        button_control.name = "MENU"
        button_control.always_visible = True
        button_control.markers += gripperMarkerList(gripperOffset)
        gripper_im.controls.append(button_control)

        gripper_im.controls.extend(make_6dof_controls())
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
    def __init__(self, arm, gripper, im_server, tf_listener):
        self._arm = arm
        self._gripper = gripper
        self._im_server = im_server
        self._tf_listener = tf_listener
        self.myObject = None
        self.frame_id = "odom"
        self.myPose = Pose(Point(1, 0, 1), Quaternion())
        self.arm_kwargs = {
            'allowed_planning_time': 15,
            'execution_timeout': 30,
            'num_planning_attempts': 5,
            'replan': False,
            'group_name': 'arm'
        }

    def start(self):
        obj = InteractiveMarker()
        self.myObject = obj
        obj.header.frame_id = self.frame_id
        obj.name = "object"
        obj.pose = self.myPose

        block_control = InteractiveMarkerControl()
        block_control.name = "Block"
        block_control.always_visible = True
        block_control.interaction_mode = InteractiveMarkerControl.MENU
        obj.menu_entries = make_menu_entries(["Take", "Drop"])
        block = Marker()
        block.type = Marker.CUBE
        block.scale.x = 0.05
        block.scale.y = 0.05
        block.scale.z = 0.10
        block.color.r = 0.0
        block.color.g = 0.5
        block.color.b = 0.5
        block.color.a = 1.0
        block_control.markers.append(block)

        dof = make_6dof_controls()
        obj.controls.extend(dof)
        obj.controls.append(block_control)

        preGrasp_control = InteractiveMarkerControl()
        preGrasp_control.name = "preGrasp"
        preGrasp_control.always_visible = True
        preGrasp_control.markers.extend(gripperMarkerList(self.preGraspPose()))
        obj.controls.append(preGrasp_control)

        grasp_control = InteractiveMarkerControl()
        grasp_control.name = "grasp"
        grasp_control.always_visible = True
        grasp_control.markers.extend(gripperMarkerList(self.graspPose()))
        obj.controls.append(grasp_control)

        lift_control = InteractiveMarkerControl()
        lift_control.name = "lift"
        lift_control.always_visible = True
        lift_control.markers.extend(gripperMarkerList(self.liftPose()))
        obj.controls.append(lift_control)

        obj.scale = 0.5

        self._im_server.insert(self.myObject, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()

    def objectPose(self):
        """Returns the current pose of the object in the odom frame"""
        return self.myPose

    def preGraspPose(self):
        """Returns the desired pose for preGrasping in the object frame"""
        pose = Pose(Point(-0.25, 0, 0), Quaternion(0, 0, 0, 1))
        return pose
        
    def graspPose(self):
        """Returns the desired pose for grasping in the object frame"""
        pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        return pose

    def liftPose(self):
        """Returns the desired pose for lifting in the object frame"""
        pose = Pose(Point(-0.05, 0, 0.2), Quaternion(0, -0.5, 0, 1))
        return pose

    def convertPose(self, pose):
        """Converts pose from object frame to odom frame"""
        objPose = self.objectPose()
        poseTranslationM = tft.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        objTranslationM = tft.translation_matrix([objPose.position.x, objPose.position.y, objPose.position.z])
        poseRotationM = tft.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        objRotationM = tft.quaternion_matrix([objPose.orientation.x, objPose.orientation.y, objPose.orientation.z, objPose.orientation.w])
        poseTransformM = np.add(np.add(poseRotationM, poseTranslationM), -1 * np.identity(4))
        objTransformM = np.add(np.add(objRotationM, objTranslationM), -1 * np.identity(4))
        # Composition of the two transformations to get from pose to parent of object frame.
        resTransformM = np.dot(objTransformM, poseTransformM)

        res = Pose()
        translation = tft.translation_from_matrix(resTransformM)
        rotation = tft.quaternion_from_matrix(resTransformM)
        res.position.x = translation[0]
        res.position.y = translation[1]
        res.position.z = translation[2]
        res.orientation.x = rotation[0]
        res.orientation.y = rotation[1]
        res.orientation.z = rotation[2]
        res.orientation.w = rotation[3]
        return res

    def _setGripperColor(self, markers, r, g, b):
        for m in markers:
            m.color.r = r
            m.color.g = g
            m.color.b = b

    def updateIK(self, control):
        markers = control.markers
        pose = markers[0].pose
        realPose = translatePose(self.convertPose(pose), Point(-0.166, 0, 0))
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose = realPose
        if self._arm.compute_ik(ps):
            rospy.loginfo('Found IK!')
            self._setGripperColor(markers, 0.0, 0.5, 0.5)
        else:
            rospy.loginfo('No IK found.')
            self._setGripperColor(markers, 1.0, 0.0, 0.0)

    def _moveToPoseName(self, poseName):
        controls = self.myObject.controls
        pose = None
        markers = None
        for c in controls:
            if c.name == poseName:
                markers = c.markers
                pose = markers[0].pose
        if pose is None:
            rospy.logerr('moveToPoseName {} failed: No pose found'.format(poseName))
            return False
        ps = PoseStamped()
        ps.header.frame_id = "odom"
        ps.pose = translatePose(self.convertPose(pose), Point(-0.166, 0, 0))
        try:
            ps = self._tf_listener.transformPose('base_link', ps)
        except (e):
            print("Couldn't convert to base_link: {}".format(e))
        error = self._arm.move_to_pose(ps, **self.arm_kwargs)
        if error is not None:
            rospy.logerr('moveToPoseName {} failed: {}'.format(poseName, error))
            self._setGripperColor(markers, 1.0, 0.0, 0.0)
            return False
        return True

    def _menuTake(self, feedback):
        self._gripper.open()
        if not self._moveToPoseName("preGrasp"):
            return False
        if not self._moveToPoseName("grasp"):
            return False
        self._gripper.close()
        if not self._moveToPoseName("lift"):
            return False
        return True

    def _menuDrop(self, feedback):
        self._gripper.close()

    def handle_feedback(self, feedback):
        self.myPose = feedback.pose
        self.myObject.pose = self.myPose
        if feedback.menu_entry_id == 1:
            self._menuTake(feedback)
        elif feedback.menu_entry_id == 2:
            self._menuDrop(feedback)
        else:
            for control in self.myObject.controls:
                if control.name in ['preGrasp', 'grasp', 'lift']:
                    self.updateIK(control)
        self._im_server.insert(self.myObject, feedback_cb=self.handle_feedback)
        self._im_server.applyChanges()



def main():
    rospy.init_node("gripper_teleop")
    wait_for_time()
    arm = robot_api.Arm()
    gripper = robot_api.Gripper()
    def shutdown():
        arm.cancel_all_goals()
    rospy.on_shutdown(shutdown)
    im_server = InteractiveMarkerServer('gripper_im_server', q_size=2)
    auto_pick_im_server = InteractiveMarkerServer('auto_pick_im_server', q_size=2)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    teleop = GripperTeleop(arm, gripper, im_server, listener)
    auto_pick = AutoPickTeleop(arm, gripper, auto_pick_im_server, listener)
    teleop.start()
    auto_pick.start()
    rospy.spin()

if __name__ == "__main__":
    main()

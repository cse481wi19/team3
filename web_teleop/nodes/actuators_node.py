#!/usr/bin/env python

import robot_api
import rospy
from web_teleop.srv import SetGripper, SetHead, SetTorso, SetArm, SetTorsoResponse


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


class ActuatorServer(object):
    def __init__(self):
        self._torso = robot_api.Torso()
        self._head = robot_api.Head()
        self._arm = robot_api.Arm()
        self._gripper = robot_api.Gripper()
        self._gripper_state = "open"

    def handle_set_torso(self, request):
        # TODO: move the torso to the requested height
        self._torso.set_height(request.height)
        return SetTorsoResponse()

    def handle_set_head(self, request):
        self._head.pan_tilt(request.pan, request.tilt)

    def handle_set_arm(self, request):
        self._arm.move_to_joints(robot_api.ArmJoints.from_list([0.0] * 7))

    def handle_set_gripper(self, request):
        if self._gripper_state == "open":
            self._gripper.close()
            self._gripper_state = "close"
        elif self._gripper_state == "close":
            self._gripper.open()
            self._gripper_state = "open"

def main():
    rospy.init_node('web_teleop_actuators')
    wait_for_time()
    server = ActuatorServer()
    torso_service = rospy.Service('web_teleop/set_torso', SetTorso,
                                  server.handle_set_torso)
    head_service = rospy.Service('web_teleop/set_head', SetHead, server.handle_set_head)
    arm_service = rospy.Service('web_teleop/set_arm', SetArm, server.handle_set_arm)
    gripper_service = rospy.Service('web_teleop/set_gripper', SetGripper, server.handle_set_gripper)
    rospy.spin()


if __name__ == '__main__':
    main()

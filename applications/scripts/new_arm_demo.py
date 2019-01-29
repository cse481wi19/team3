#! /usr/bin/env python

import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    argv = rospy.myargv()
    if len(argv) < 8:
        print("ARGS!!!!!")
        exit()
    vals = []
    for i in range(1, 8):
        vals.append(float(argv[i]))

    rospy.init_node('arm_demo')
    wait_for_time()

    torso = robot_api.Torso()
    torso.set_height(robot_api.Torso.MAX_HEIGHT)

    arm = robot_api.Arm()
    arm.move_to_joints(robot_api.ArmJoints.from_list(vals))


if __name__ == '__main__':
    main()


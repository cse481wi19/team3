#! /usr/bin/env python

import robot_api
import rospy


def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass


def main():
    rospy.init_node('limp_arm')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 2:
        print "NO YOU FOOL!"
        return
    arm = robot_api.Arm()
    if argv[1] == "limp":
        arm.relax_arm()
    else:
        arm.enable_arm()

if __name__ == "__main__":
    main()

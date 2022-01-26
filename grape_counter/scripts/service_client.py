#!/usr/bin/env python

import rospy
from grape_counter.srv import SetMode


def set_mode_client(mode):
    rospy.wait_for_service('set_mode')
    try:
        set_mode_srv = rospy.ServiceProxy('set_mode', SetMode)
        resp1 = set_mode_srv(mode)
        return resp1.result
    except rospy.ServiceException as e:
        print("Service call failed: ", e)


if __name__ == "__main__":
    print("Requesting...")
    res = set_mode_client(1)
    print("Result: Mode is ", res)
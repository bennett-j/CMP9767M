#!/usr/bin/env python

from grape_counter.srv import SetMode, SetModeResponse
import rospy


def handle_set_mode(req):
    print("Mode set to: ", req.mode, " by server")
    return SetModeResponse(req.mode)


def set_mode_server():
    rospy.init_node('set_mode')
    rospy.Service('set_mode', SetMode, handle_set_mode)
    print("Ready to add set mode.")
    rospy.spin()

if __name__ == "__main__":
    set_mode_server()
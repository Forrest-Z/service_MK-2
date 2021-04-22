#!/usr/bin/env python

from zetabot_main.srv import Dbsrv
import sys
import rospy

def add_two_ints_client():
    rospy.wait_for_service('srv2')
    try:
        add_two_ints = rospy.ServiceProxy('srv2', Dbsrv)
        add_two_ints("asdf", "dfafd")
        print("eeeeeee")
        return 0
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    add_two_ints_client()

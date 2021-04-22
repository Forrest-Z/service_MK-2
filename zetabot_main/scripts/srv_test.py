#!/usr/bin/env python

from zetabot_main.srv import MoveBaseSrv
from zetabot_main.srv import Dbsrv
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.x, req.y, (req.x + req.y)))
    result = str(req.x + req.y)
    rospy.sleep(10)
    return result

def srv22(req):
    print(req.command)
    return "aaa"

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', MoveBaseSrv, handle_add_two_ints)
    srv2 = rospy.Service("srv2",Dbsrv,srv22)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
#!/usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from zetabot_main.msg import MoveMsgs, MoveBaseActAction, MoveBaseActFeedback, MoveBaseActResult

from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def test (msg) :
    msg_str = str(msg)
    msg_arr = msg_str.split(".")[-1].split("'")[0]
    msg_Feedback = msg_arr.replace("Action","Feedback")
    msg_Result = msg_arr.replace("Action","Result")

    __feedback = globals()[msg_Feedback]()
    __Result = globals()[msg_Result]()
    print(__feedback)
    print(__Result)

test(MoveBaseActAction)

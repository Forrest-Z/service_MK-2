#! /usr/bin/env python

import rospy
# from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import zetabot_main.msg

from zetabot_main.msg import MoveMsgs, MoveBaseActAction, MoveBaseActFeedback, MoveBaseActResult, MoveBaseActGoal

from std_srvs.srv import Empty

def movebase_client(x,y,z):
    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('ai_move', MoveBaseActAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()


    # Creates a goal to send to the action server.
    goal = zetabot_main.msg.MoveBaseActGoal()
    goal.header.frame_id = "air_condition"
    goal.x, goal.y, goal.z = x, y, z

    # Sends the goal to the action server.
    client.send_goal(goal)
    print("send_goal")
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print("get_result")
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = movebase_client(5.54425404643,-0.898580809085,0.1)
        print("Result :    ", result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
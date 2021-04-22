#! /usr/bin/env python
import rospy

import os

import actionlib
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from zetabot_main.msg import MoveMsgs
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from zetabot_main.srv import ModuleControllerSrv
from std_srvs.srv import Empty
import my_first_ros_pkg.msg
from geometry_msgs.msg import PoseWithCovarianceStamped

index = 0

module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

def movebase_client(x,y,z=1):


    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

def roming_move() :
    global index
    print("call_back")
    # roming_list = [  
    #     {"x" : 6.0, "y" : 1.46 },
    #     {"x" : 6.42, "y" : -2.81 },
    #     {"x" : 2.8, "y" : 1.46 },
    #     {"x" : 3.15, "y" : -2.6 }
    #     ] ###gwang ju 
    
    roming_list = [
        {"x" : 0.42, "y" : -1.74 },
        {"x" : 4.35, "y" : 0.023 },
        {"x" : 3.316, "y" : 2.301 },
        {"x" : 0.66, "y" : -0.66 }
        ] #hub
           
    result = movebase_client(roming_list[index]["x"],roming_list[index]["y"])
    
    index += 1
    if index >= 4 :
        index = 0


module_controller_srv("air_lv2_on")

while True:
    rospy.init_node("robing")
    roming_move()
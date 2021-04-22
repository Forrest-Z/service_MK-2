#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import socket
import json
import threading
import math
import actionlib
import time
import os
import sys
from full_coverage.srv import Cell2pose
from zetabot_main.srv import Dbsrv
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose

from std_srvs.srv import Empty


pose = Pose()

visited_list = []

class_list = ['alldata_a','alldata_b','alldata_c']

robot_z = 0

cell2pose = rospy.ServiceProxy("cell2pose",Cell2pose)

now_cell_name = ""

# threading._start_new_thread(sendChat, ())

def angle_scailing(z) :
    z = z-90

    if z<0 :
        return z + 360

    else :
        return z

def quaternion_to_euler_angle(msg):
    x = msg.x
    y = msg.y
    z = msg.z
    w = msg.w

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def g_rangle_range_tr(euler_z):

    angle = 0
    if euler_z > 0:
        angle = euler_z
    else:
        angle = 180 + euler_z + 179

    return angle

def pose_send(val) :
    global pose
    global robot_z

    pose = val
    X, Y, Z = quaternion_to_euler_angle(pose.orientation)
    robot_z_comp = g_rangle_range_tr(Z)
    robot_z = angle_scailing(robot_z_comp)

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

def cell_name_recv(msg) :
    global now_cell_name
    now_cell_name = msg.data

def near_cell_name_get(cell_data):
    global now_cell_name

    cell_distance = {}

    for i in cell_data :
        if not i["cell_name"] == now_cell_name :
            cell_axis = cell2pose(i['cell_name'])
            cell_distance[i['cell_name']] = math.sqrt((cell_axis.x - pose.position.x)**2 + (cell_axis.y - pose.position.y)**2)

    near_cell_name = min(cell_distance, key=cell_distance.get)

    return near_cell_name

def class_classification(cell_data,msg) :
    classificated_cell_list = list()

    for i in cell_data :
        if i["class"] == msg[-1] :
            classificated_cell_list.append(i)
    return classificated_cell_list

def local_cell_condition(msg,call_name = False):
    today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
    local_data = {}

    # local_data = {
    #     "a1" : {
    #         "cell_name" : "a1",
    #         "x"         : 0.1,
    #         "y"         : 0.1,
    #         "class"     : "a"
    #     },
    #     "a2" : {
    #         "cell_name" : "a2",
    #         "x"         : 0.2,
    #         "y"         : 0.2,
    #         "class"     : "b"
    #     },
    #     "a3" : {
    #         "cell_name" : "a3",
    #         "x"         : 0.3,
    #         "y"         : 0.3,
    #         "class"     : "b"
    #     }
    # }
    


    if msg == "cell_info" :
        file_path = os.path.dirname( os.path.abspath( __file__ ) ) + "/map/"+msg+".json"

    else : 
        file_path = os.path.dirname( os.path.abspath( __file__ ) ) + "/map/air_class_"+today+".json"


    try :
        # print(file_path)
        with open(file_path, 'r') as json_file:
            local_data = json.load(json_file)

    except :
        print("local_file_load_Err")
    
    if call_name :
        for i in local_data :
            if msg == i["cell_name"]:
                return i["class"]
    
    if not msg == "cell_info" :

        cell_class_info = class_classification(local_data,msg)

        return cell_class_info

    else : 
        return local_data

def air_conditioning(near_cell_name,cell_class) :
    during_time = 10
    minimum_stay_flag = False

    start_time = time.time()
    # print("near_cell_name :", near_cell_name)
    cell_class_info = local_cell_condition(near_cell_name,True)


    # while time.time() - start_time < (during_time-1) *60 :
    while time.time() - start_time < 0.1 :
        sys.stdout.write("air_class : " + str(cell_class_info) + "\r")
        if cell_class > cell_class_info or cell_class_info == "a" :
            while time.time() - start_time < 2:
                pass
    sys.stdout.write("air_class : " + str(cell_class_info))
    return 0 
    
def roaming_cell(cell_data,cell_class):
    global visited_list
    global cell2pose
        

    near_cell_name = near_cell_name_get(cell_data)
    near_cell = cell2pose(near_cell_name)
    print("go to   ",near_cell_name,"x: ",near_cell.x,"y: ",near_cell.y)
    result = movebase_client(near_cell.x,near_cell.y)
    print("stay to ",near_cell_name)
    air_conditioning(near_cell_name,cell_class)
    visited_list.append(near_cell_name)
    same_cell_idx = []
    idx = 0
    for i in cell_data :
        if i["cell_name"] in visited_list :
            same_cell_idx.append(idx)
        idx = idx +1 
    for i in same_cell_idx :
        del cell_data[i]

    return cell_data

def air_conditioning_mode():
    global now_cell_name

    while not now_cell_name : 
        pass
    for i in reversed(class_list) :
        cell_data = call_cell_condition(i)
        idx = 0
        for j in cell_data :
            if j["cell_name"] == now_cell_name :
                del cell_data[idx]
            idx = idx + 1
        # print(cell_data)

        while True:
            if not cell_data :
                break
            os.system('clear')
            print("remain cell : ", cell_data)
            print("="*10,"class : ", i[-1], "="*10)
            cell_data = roaming_cell(cell_data,i)
            

def call_cell_condition(msg) :

    cell_data = {}

    call_cell_condition_service_name = 'db_works'
    # rospy.wait_for_service(call_cell_condition_service_name)
    
    try :
        call_cell_condition_service = rospy.ServiceProxy(call_cell_condition_service_name,Dbsrv)
        cell_condition = call_cell_condition_service(msg,"")
        cell_result = cell_condition.result.replace("'",'"')
        cell_data = json.loads(cell_result)
    except:
        print("Cell_condition call failed")

    if not cell_data :
        
        cell_data = local_cell_condition(msg)

    return cell_data


def main() :
    global visited_list

    rospy.init_node('cell_condition_client')

    robot_pose_topic = "/robot_pose"
    rospy.Subscriber(robot_pose_topic,Pose,pose_send)

    cell_name_topic = "/cell_name"
    rospy.Subscriber(cell_name_topic,String,cell_name_recv)

    air_conditioning_mode()
    print("air conditionig done")
    visited_list = []
    rospy.sleep(2)


    rospy.spin()


if __name__ == "__main__" :

    main()
#!/usr/bin/env python
"""
    Zeta robot auto-return to charge station protocol between control module and camera/opencv module.
    Client Program : running on control module
    (using Python3 --> 2.7) 

    History:
        20200410 kyuhsim - make not to use tcp/ip. just call direction_api directly
        20200304 kyuhsim - change for Python2.7
        20200207 kyuhsim - zeta autoreturn protocol
"""

# Python 2/3 compatibility
from __future__ import print_function

import sys
from time import time, sleep
import socket
from threading import Thread
import json
from direction_api import Direction
from collections import Counter

#ros rib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt64
from std_msgs.msg import Char
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import math

#ros simple goal lib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from std_srvs.srv import Empty

#from Tkinter import *

# ## global variables
# ## ros global
target_linear_vel = 0
target_angular_vel = 0
control_linear_vel = 0
control_angular_vel = 0

g_rangle = 0
now_g_rangle = 0
target_g_rangle = 0
now_degree = 0
stop_time = 0

process_th = False
stop_flag = False
turn_mode = False
center_flag = False
set_back = False
contact_state_flag = False
R_contact_state_flag = False
L_contact_state_flag = False
stop_flag = True
charge_state = True

contact_state = ""
battery_amount = ""
seq = ""
system_command = ""

check_direction_list = []
get_direction_list = []
amount_list = [[], []]

def init_list():
    global check_direction_list
    global get_direction_list
    global amount_list

    check_direction_list = ['0' for col in range(3)]
    get_direction_list = ['0' for col in range(7)]
    amount_list = [[0 for col in range(10)] for row in range(3)]

#simple goal
def movebase_client(x, y):
    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# ## for robotheading angle
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

def main_system_command_subscriber_callback(msg):
    global system_command
    global process_th
    global seq

    system_command = msg.data

    if system_command == "go":
        movebase_client(-0.260, -2.108)
        seq = "start"
        
    else :
        pass


def imu_callback(msg):
    X, Y, Z = quaternion_to_euler_angle(msg.orientation)
    g_rangle_range_tr(Z)

    '''
    print ("Quaternoion ================")
    print ("x : ", msg.orientation.x)
    print ("y : ", msg.orientation.y)
    print ("z : ", msg.orientation.z)
    print ("w : ", msg.orientation.w)
    print ("Euler -----------------------")
    print ("X : ", X)
    print ("Y : ", Y)
    print ("Z : ", Z)
    print("g_rangle : ", g_rangle)
    '''

def battery_amount_subscriber_callback(msg):
    global battery_amount

    battery_amount = int(msg.data)

def INO_state_subscriber_callback(msg):
    global contact_state
    global R_contact_state_flag
    global L_contact_state_flag
    global contact_state_flag

    contact_state = msg.data

    if contact_state == "contact":
        contact_state_flag = True
        L_contact_state_flag = True
        R_contact_state_flag = True

    elif contact_state == "left":
        contact_state_flag = False
        L_contact_state_flag = True
        R_contact_state_flag = False

    elif contact_state == "right":
        contact_state_flag = False
        L_contact_state_flag = False
        R_contact_state_flag = True

    elif contact_state == "not_connected":
        contact_state_flag = False
        L_contact_state_flag = False
        R_contact_state_flag = False

    elif contact_state == "start":
        contact_state_flag = False
        L_contact_state_flag = False
        R_contact_state_flag = False


def NUC_state_publisher(step_num):
    NUC_state_pub = rospy.Publisher('autocharge_state_NUC', UInt8, queue_size = 1)
    pub_rate = rospy.Rate(25) #5hz

    rospy.loginfo(step_num)
    NUC_state_pub.publish(step_num)
    pub_rate.sleep()
'''
def charge_time_publisher():
    charge_time_pub = rospy.Publisher('charge_time', UInt64, queue_size = 1)
    pub_rate = rospy.Rate(30) #30hz

    ptime = int(rospy.get_time() * 100)
    rospy.loginfo(ptime)
    charge_time_pub.publish(ptime)
    pub_rate.sleep()
'''
def left_turn(x):
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel

    control_angular_vel = x

def right_turn(x):
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel

    control_angular_vel = -(x)

def stop_turn():
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel

    target_linear_vel = 0
    control_linear_vel = 0
    target_angular_vel = 0
    control_angular_vel = 0

def for_linear(x):
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel

    target_linear_vel = 0
    control_linear_vel = 0
    target_angular_vel = 0
    control_angular_vel = 0
    sleep(2)

    forward_time = int(float(x))
    control_linear_vel = 0.026
    sleep(forward_time)

def bak_linear(x, y):
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel

    control_linear_vel = x
    control_angular_vel = y

def pubTimer():
    global target_linear_vel
    global control_linear_vel
    global target_angular_vel
    global control_angular_vel
    global process_th
    while True :
        while  process_th:
            
            twist = Twist()

            twist.linear.x = control_linear_vel
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = control_angular_vel

            velocity_pub.publish(twist)
            sleep(0.1)

def stop_delay():
    global stop_flag
    global stop_time

    if stop_time < 5:
        stop_turn()
        stop_time += 1
        sleep(0.05)
        print("stop count: ", stop_time)
    else:
        stop_flag = False
        stop_time = 0

#g_rangle range 0 ~ 359
def g_rangle_range_tr(euler_z):
    global g_rangle
    g_rangle = 0
    if euler_z > 0:
        g_rangle = int(euler_z) + 180
    else:
        g_rangle = int(euler_z) + 179

def left_direction_turn(now_degree):
    global g_rangle
    global target_g_rangle
    global now_g_rangle

    target_degree = 90 + now_degree
    now_g_rangle = g_rangle

    if now_g_rangle >= target_degree :
        target_g_rangle = now_g_rangle - target_degree
    elif now_g_rangle < target_degree :
        target_g_rangle = (now_g_rangle + 360) - target_degree

    while True :
        '''Left_Turning'''
        NUC_state_publisher(4)
        print("now_g_rangle : ", now_g_rangle, ", target_g_rangle : ", target_g_rangle, ", g_rangle : ", g_rangle, ", target_degree : ", target_degree, ", now_degree : ", now_degree)
        right_turn(0.25)
        if (target_g_rangle - 1) == g_rangle :
            break

def right_direction_turn(now_degree):
    global g_rangle
    global target_g_rangle
    global now_g_rangle

    target_degree = 90 + now_degree
    now_g_rangle = g_rangle

    if now_g_rangle <= 359 - target_degree:
        target_g_rangle = now_g_rangle + target_degree
    elif now_g_rangle > 359 - target_degree:
        target_g_rangle = (now_g_rangle + target_degree) - 359

    while True :
        '''Right_Turning'''
        NUC_state_publisher(4)
        print("now_g_rangle : ", now_g_rangle, ", target_g_rangle : ", target_g_rangle, ", g_rangle : ", g_rangle, ", target_degree : ", target_degree, ", now_degree : ", now_degree)
        left_turn(0.25)
        if (target_g_rangle - 1) == g_rangle :
            break

if __name__ == '__main__':

    if len(sys.argv) != 1:
        print("- Usage : python {}".format(sys.argv[0]))
        print("* Exiting...")
    else:
        api = Direction()

        api.init()
        api.debug_show_img(False)
        api.check_direction()    # discard to settle up camera
        api.debug_show_img(True)

        battery_amount_subscriber = rospy.Subscriber('battery', String, battery_amount_subscriber_callback)
        INO_state_subscriber = rospy.Subscriber('autocharge_state_INO', String, INO_state_subscriber_callback)
        main_system_subscriber = rospy.Subscriber('main_system_command', String, main_system_command_subscriber_callback)
        imu_sub = rospy.Subscriber('imu', Imu, imu_callback)

        rospy.init_node('zetabank_autocharge')
        velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
        thread1 = Thread(target = pubTimer)
        thread1.start()
        '''thread1.daemon = True'''

        init_list()
        seq = "waiting"
        print("init setting -----------------")
        sleep(1.3)
        
        # Using function call

        while True:
            '''________________First priority________________'''
            #charge_time_publisher()

            if stop_flag == True:
                stop_delay()


            '''________________Main sequence________________'''
            if seq == "waiting":
                NUC_state_publisher(0)
                sleep(1)

            elif seq == "start":
                NUC_state_publisher(1)

                if contact_state == "start":
                    seq = "search"



            elif seq == "search":
                process_th = True
                NUC_state_publisher(2)
                result = api.check_direction()
                result_center = api.check_center(5)

                check_direction_list = check_direction_list[1:]
                check_direction_list.append(result)
                result_direction = Counter(check_direction_list).most_common(2)

                if stop_flag == False:
                    if result_direction[0][0] == 'fail':
                        if turn_mode == False:
                            left_turn(0.15)
                        elif turn_mode == True:
                            right_turn(0.15)

                    elif result_direction[0][0] == 'right':
                        right_turn(0.05)

                    elif result_direction[0][0] == 'left':
                        left_turn(0.05)

                    elif result_direction[0][0] == 'success':
                        charge_state = False
                        result_center = api.check_center(4)
                        if result_center == 'left':
                            left_turn(0.02)
                        elif result_center == 'right':
                            right_turn(0.02)
                        elif result_center == 'success':
                            stop_turn()
                            stop_flag = True
                            seq = "adjustment"
                        else: pass

                    else: pass

                process_th = False



            elif seq == "adjustment":
                process_th = True
                NUC_state_publisher(3)
                result, direction, amount, distance, degree = api.get_direction()

                if direction == '-': pass
                else:
                    get_direction_list = get_direction_list[1:]
                    del amount_list[0][0]
                    del amount_list[1][0]
                    del amount_list[2][0]

                    get_direction_list.append(direction)
                    amount_list[0].append(int(float(amount)))
                    amount_list[1].append(int(float(degree)))
                    amount_list[2].append(int(float(distance)))

                    result_direction = Counter(get_direction_list).most_common(2)
                    avg_amount = sum(amount_list[0], 0.0) / len(amount_list[0])
                    avg_degree = sum(amount_list[1], 0.0) / len(amount_list[1])
                    avg_distance = sum(amount_list[2], 0.0) / len(amount_list[2])

                if stop_flag == False:
                    if result == 'fail':
                        stop_turn()
                        stop_flag = True
                        seq = "search"

                    elif result == 'success':
                        if result_direction[0][0] == 'left' and center_flag == False:
                            turn_mode = False
                            now_degree = int(float(avg_degree))
                            left_direction_turn(now_degree)
                            for_linear(avg_amount+1)
                            init_list()
                            stop_turn()
                            stop_flag = True
                        elif result_direction[0][0] == 'right' and center_flag == False:
                            turn_mode = True
                            now_degree = int(float(avg_degree))
                            right_direction_turn(now_degree)
                            for_linear(avg_amount+1)
                            init_list()
                            stop_turn()
                            stop_flag = True
                        elif result_direction[0][0] == 'center':
                            center_flag = True
                            bak_linear(-0.01, 0)
                            seq = "guidance"    
                        else: pass

                    else: pass
                process_th = False



            elif seq == "guidance":
                process_th = True
                NUC_state_publisher(5)
                result, direction, amount, distance, degree = api.get_direction()

                print("contact_state_flag : ", contact_state_flag)

                if contact_state_flag == True:
                    stop_turn()
                    stop_flag = True
                    seq = "charging"

                if direction == '-': pass
                else: 
                    get_direction_list = get_direction_list[1:]
                    get_direction_list.append(direction)
                    result_direction = Counter(get_direction_list).most_common(2)
 
                if stop_flag == False:
                    if result == 'fail':
                        if set_back == False:
                            stop_turn()
                            stop_flag = True
                            seq = "guidance"
                        elif set_back == True:
                            if L_contact_state_flag == True and R_contact_state_flag == False:
                                bak_linear(-0.008, 0.002)
                            elif L_contact_state_flag == False and R_contact_state_flag == True:
                                bak_linear(-0.008, -0.002)
                            elif L_contact_state_flag == False and R_contact_state_flag == False:
                                bak_linear(-0.008, 0)

                    elif result == 'success':
                        set_back = True
                        result_center = api.check_center(3)

                        if result_center == 'left':
                            bak_linear(-0.008, 0.002)
                        elif result_center == 'right':
                            bak_linear(-0.008, -0.002)
                        elif result_center == 'center':
                            bak_linear(-0.008, 0)
                        else: pass

                    else: pass
                process_th = False

            elif seq == "charging":
                process_th = True
                NUC_state_publisher(6)
                print("contact_state_flag : ", contact_state_flag)
                print("L_contact_state_flag : ", L_contact_state_flag)
                print("R_contact_state_flag : ", R_contact_state_flag)
                if contact_state_flag == False:
                    seq = "not_connected"

                print("battery_amount : ", battery_amount)
                if battery_amount >= 1200:
                    seq = "finish"
                process_th = False



            elif seq == "finish":
                process_th = False
                NUC_state_publisher(7)
                print("contact_state_flag : ", contact_state_flag)
                print("L_contact_state_flag : ", L_contact_state_flag)
                print("R_contact_state_flag : ", R_contact_state_flag)
                # autonous driving

                print("battery_amount : ", battery_amount)
                print(battery_amount)
                if battery_amount < 500:
                    seq = "charging"



            elif seq == "not_connected":
                process_th = True
                NUC_state_publisher(8)
                print("contact_state_flag : ", contact_state_flag)
                print("L_contact_state_flag : ", L_contact_state_flag)
                print("R_contact_state_flag : ", R_contact_state_flag)
                for_linear(8)
                stop_turn()
                seq = "start"
                process_th = False


            else: 
                process_th = False

        print("- Program exiting normally")

# #### End of File #####


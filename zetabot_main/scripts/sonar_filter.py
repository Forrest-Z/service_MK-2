#!/usr/bin/env python
import rospy
import roslib
import actionlib

import sys
import os
import math
import numpy
import threading
import time

import full_coverage.msg
from std_msgs.msg import UInt8
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Range
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
from std_msgs.msg import String
from full_coverage.msg import lidar_filter 
from zetabot_main.srv import ModuleControllerSrv
from zetabot_main.msg import SonarArray
from sensor_msgs.msg import Imu


sonar_warning_top = 40
sonar_warning_bottom = 35

sonar_stop_top = 30
sonar_stop_bottom = 25

wall_distance = 0.25
max_a_val = 0.2
x_val = 0.1

pose = Pose()
robot_z = 0
twist = Twist()
key = 1
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

bumper_flag = False
estop_flag = False
MVStop_flag = False
imu_avr_term = None
imu_get_list = []
imu_get_time = None
stm_called = True
stm_recall_flag = False



emergency_topic = "/emergency_stop"
emergency_pub = rospy.Publisher(emergency_topic,String, queue_size=10)
emergency_call = rospy.ServiceProxy("/emergency_stm", Empty)
module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

rospy.init_node('sonar_filter')

class sonar_sensor :
    sonar_UF = 0
    sonar_UR = 0
    sonar_UB = 0
    sonar_UL = 0
    sonar_DF = 0
    sonar_DR1 = 0
    sonar_DR2 = 0
    sonar_DB1 = 0
    sonar_DB2 = 0
    sonar_DL1 = 0
    sonar_DL2 = 0

def recv_sonar(data):
    sonar_sensor.sonar_UF = data.data[5]
    sonar_sensor.sonar_UR = data.data[6]
    sonar_sensor.sonar_UB = data.data[7]
    sonar_sensor.sonar_UL = data.data[10]
    sonar_sensor.sonar_DF = data.data[4]
    sonar_sensor.sonar_DR1 = data.data[1]
    sonar_sensor.sonar_DR2 = data.data[0]
    sonar_sensor.sonar_DB1 = data.data[9]
    sonar_sensor.sonar_DB2 = data.data[8]
    sonar_sensor.sonar_DL1 = data.data[2]
    sonar_sensor.sonar_DL2 = data.data[3]

    # os.system('clear')
    # for key,value in sonar_sensor.items() : 
    #     print(key + " : " + str(value))
    

    # print("-"*30)
    # print(data)
    # print("-"*30)


def recv_bumper(data):
    global bumper_flag
    bumper_flag = data.data

def recv_estop(data):
    global estop_flag
    estop_flag = data.data

def recv_MBStop(data):
    global MVStop_flag
    MVStop_flag = data.data

def recv_imu(data):
    global imu_avr_term
    global imu_get_list
    global imu_get_time
    global stm_called
    global stm_recall_flag
    stm_called = True
    stm_recall_flag = False


    if imu_avr_term == None :
        imu_get_list.append(rospy.get_time())
        if len(imu_get_list) >= 5 :
            imu_avr_term = (imu_get_list[-1] - imu_get_list[0]) / 4
            imu_get_time = imu_get_list[-1]

    else :
        imu_get_time = rospy.get_time()

def emergency_stm_call():
    emergency_call()

def emergency_send () :
    global bumper_flag
    global estop_flag
    global stm_called
    global stm_recall_flag
    
    while True:
        rospy.sleep(0.01)
        emergency_msg = ""
        os.system("clear")
        # print("sonar_UF :",sonar_sensor.sonar_UF)
        # print("sonar_UR :",sonar_sensor.sonar_UR)
        # print("sonar_UB :",sonar_sensor.sonar_UB)
        # print("sonar_UL :",sonar_sensor.sonar_UL)
        # print("sonar_DF :",sonar_sensor.sonar_DF)
        # print("sonar_DR1 :",sonar_sensor.sonar_DR1)
        # print("sonar_DR2 :",sonar_sensor.sonar_DR2)
        # print("sonar_DB1 :",sonar_sensor.sonar_DB1)
        # print("sonar_DB2 :",sonar_sensor.sonar_DB2)
        # print("sonar_DL1 :",sonar_sensor.sonar_DL1)
        # print("sonar_DL2 :",sonar_sensor.sonar_DL2)

        print("estop_flag : ",estop_flag)

        if bumper_flag == True : 
            emergency_msg += "/bumper_stop"

        if estop_flag == True :
            emergency_msg += "/emergency_button_stop"

        if MVStop_flag == True :
            emergency_msg += "/move_base_stop"

        if imu_avr_term != None :
            print(rospy.get_time() - imu_get_time)
            print(imu_avr_term)
            print(imu_avr_term + (imu_avr_term/2))
            print(imu_get_list)
        if imu_avr_term == None :
            emergency_msg += "/imu_error_stop"


        elif rospy.get_time() - imu_get_time >= (imu_avr_term*10) :
            # os.system("rosnode kill /stm_starter")
            emergency_msg += "/imu_error_stop"
            stm_called = False

        if not stm_called and not stm_recall_flag:
            stm_recall_flag = True
            print("recall")
            #stm call thread
            
            #set_param odom_sub.msg

            t1 = threading.Thread(target=emergency_stm_call)
            t1.daemon = True 
            t1.start()

            print("return")
        # if (sonar_sensor.sonar_UF <= sonar_stop_top or sonar_sensor.sonar_DF <= sonar_stop_bottom  ) :
        #     emergency_msg += "/sonar_stop"
        #     print("sonar_sensor_val :",sonar_sensor.sonar_UF)
        #     print("sonar_stop")

        # elif (sonar_sensor.sonar_UF <= sonar_warning_top or sonar_sensor.sonar_DF <= sonar_warning_bottom  ) :
        #     emergency_msg += "/sonar_warning"
        #     print("sonar_sensor_val :",sonar_sensor.sonar_UF)
        #     print("sonar_warning")

        emergency_pub.publish(emergency_msg)

def main():

    t1 = threading.Thread(target=emergency_send)
    t1.daemon = True 
    t1.start()

    # sonar_topic = "/sonar"
    # rospy.Subscriber(sonar_topic,SonarArray,recv_sonar)

    bumper_topic = "/bumper"
    rospy.Subscriber(bumper_topic,Bool,recv_bumper)

    estop_topic = "/estop"
    rospy.Subscriber(estop_topic,Bool,recv_estop)

    move_base_stop_topic = "/move_base_stop"
    rospy.Subscriber(move_base_stop_topic,Bool,recv_MBStop)

    imu_topic = "/imu"
    rospy.Subscriber(imu_topic,Imu,recv_imu)
    
    rospy.spin()

if __name__ == "__main__":
    main()
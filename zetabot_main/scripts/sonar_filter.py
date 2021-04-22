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

emergency_topic = "/emergency_stop"
emergency_pub = rospy.Publisher(emergency_topic,String, queue_size=10)

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

def emergency_send () :
    global bumper_flag
    global estop_flag

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
    
    rospy.spin()

if __name__ == "__main__":
    main()
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

sonar_sensor = dict()


sonar_front1 = "sonar1"
sonar_front2 = "sonar2" 
sonar_right1 = "sonar3"
sonar_right2 = "sonar4"
sonar_rear1 = "sonar5"
sonar_rear2 = "sonar6"
sonar_left1 = "sonar7"
sonar_left2 = "sonar8"

sonar_list = {
    "sonar_UF" : "sonar1",
    "sonar_UR" : "sonar2",
    "sonar_UB" : "sonar3",
    "sonar_UL" : "sonar4",
    "sonar_DF" : "sonar5",
    "sonar_DR1" : "sonar6",
    "sonar_DR2" : "sonar7",
    "sonar_DB1" : "sonar8",
    "sonar_DB2" : "sonar9",
    "sonar_DL1" : "sonar10",
    "sonar_DL2" : "sonar11",

}


sonar_warring_top = 0.36
sonar_warring_bottom = 0.30

sonar_stop_top = 0.16
sonar_stop_bottom = 0.10

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

emergency_topic = "/emergency_stop"
emergency_pub = rospy.Publisher(emergency_topic,String, queue_size=10)

module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

rospy.init_node('sonar_filter')

def recv_sonar(data):
    global sonar_sensor
    sonar_sensor[data.header.frame_id] = data.range
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

def emergency_send () :
    global bumper_flag
    global estop_flag
    global sonar_sensor

    while len(sonar_sensor.keys()) != 11 :
        None
    while True:
        emergency_msg = ""

        if bumper_flag == True : 
            emergency_msg += "/bumper_stop"

        if estop_flag == True :
            emergency_msg += "/emergency_button_stop"

        if (sonar_sensor[sonar_list["sonar_UF"]] <= sonar_warring_top or sonar_sensor[sonar_list["sonar_DF"]] <= sonar_warring_bottom  ) :
            emergency_msg += "/sonar_warring"

        elif (sonar_sensor[sonar_list["sonar_UF"]] <= sonar_stop_top or sonar_sensor[sonar_list["sonar_DF"]] <= sonar_stop_bottom  ) :
            emergency_msg += "/sonar_stop"

        emergency_pub.publish(emergency_msg)

def main():

    t1 = threading.Thread(target=emergency_send)
    t1.daemon = True 
    t1.start()

    sonar_topic = "/sonar"
    rospy.Subscriber(sonar_topic,Range,recv_sonar)

    bumper_topic = "/bumper"
    rospy.Subscriber(bumper_topic,Bool,recv_bumper)

    estop_topic = "/estop"
    rospy.Subscriber(estop_topic,Bool,recv_estop)
    
    rospy.spin()

if __name__ == "__main__":
    main()
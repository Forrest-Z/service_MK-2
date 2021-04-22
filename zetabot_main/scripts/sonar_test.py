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
    "sonar_UF" : "sonar3",
    "sonar_UR" : "sonar2",
    "sonar_UB" : "sonar1",
    "sonar_UL" : "sonar4",
    "sonar_DF" : "sonar5",
    "sonar_DR1" : "sonar6",
    "sonar_DR2" : "sonar7",
    "sonar_DB1" : "sonar8",
    "sonar_DB2" : "sonar9",
    "sonar_DL1" : "sonar10",
    "sonar_DL2" : "sonar11
}


rospy.init_node('sonar_test')

def recv_sonar(data):
    global sonar_sensor
    sonar_sensor[data.header.frame_id] = data.range

    os.system('clear')
    # for key,value in sonar_sensor.items() : 
    #     print(key + " : " + str(value))

    print("sonarUF : "+str(sonar_sensor[sonar_list["sonar_UF"]]))
    print("sonarUR : "+str(sonar_sensor[sonar_list["sonar_UR"]]))    
    print("sonarUB : "+str(sonar_sensor[sonar_list["sonar_UB"]]))    
    print("sonarUF : "+str(sonar_sensor[sonar_list["sonar_UL"]]))    

    print("-"*30)
    print("-"*30)




def main():

    sonar_topic = "/sonar"
    rospy.Subscriber(sonar_topic,Range,recv_sonar)
    
    rospy.spin()

if __name__ == "__main__":
    main()
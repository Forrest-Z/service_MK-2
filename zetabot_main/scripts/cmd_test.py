#!/usr/bin/env python
import roslib

from full_coverage.srv import Fullpath
import rospy
import math
import numpy
import threading
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from full_coverage.msg import lidar_filter
from full_coverage.srv import Cell2pose
from zetabot_main.srv import ModuleControllerSrv



def twist_init() :
    global twist

    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0



def main():
    global timer
    global twist
    
    rospy.init_node('cmd_test')

    twist = Twist()

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    val = 0.03

    while True : 
        twist.linear.x = val
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.5)
        val = val * (-1)

    rospy.spin()

if __name__ == "__main__":
    main()
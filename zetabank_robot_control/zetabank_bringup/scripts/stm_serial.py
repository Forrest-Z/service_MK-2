#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os,sys,threading, time
from sensor_msgs.msg import Imu

class Ros_check:
    def __init__(self, imu_topic):
        self.check_flag = False
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.callback)
    def callback(self, msg):
        self.check_flag = True

class Command:
    def __init__(self):
        self.command_rosrun = "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM{} _baud:=460800 __name:='STM'"
    def ACM_check(self):
        for i in range(2):
            command_rosrun = self.command_rosrun.format(i)
            t1 = threading.Thread(target= ACM_thread, name= "ACM thread", args=(command_rosrun,))
            t1.setDaemon(True)
            t1.start()
            #ros
            imu_topic = '/imu'
            ros_check = Ros_check(imu_topic)
            time.sleep(5)

            if not ros_check.check_flag:
                nodes = os.popen("rosnode lsit").readline()
                os.system("rosnode kill /STM")
                t1.join()
            else:
                break
                

def ACM_thread(command_rosrun):
    os.system(command_rosrun)


if __name__=='__main__':
    rospy.init_node("stm_starter")
    Command().ACM_check()
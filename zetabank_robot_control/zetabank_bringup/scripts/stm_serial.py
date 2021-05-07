#!/usr/bin/env python
import rospy
import os

rospy.init_node("stm_starter")

os.system("rosrun rosserial_python serial_node.py _baud:=460800 _port:=/dev/$(ls -lrt /dev | grep ^l | grep stlinkv2-1 |grep ttyACM | awk '{print$(NF-2)}') __name:='stm'")
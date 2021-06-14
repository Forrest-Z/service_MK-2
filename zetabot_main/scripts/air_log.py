#! /usr/bin/env python

import rospy

import csv
import time
import sys, os
from zetabot_main.msg import EnvironmentMsgs
from geometry_msgs.msg import Pose


air_log = EnvironmentMsgs()
pose_log = Pose()
file_ready_flag = False
pose_ready_flag = False

battery_cnt = 2

today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

log_directory = "/home/zetabank/robot_log/air_log"
file_name = log_directory + "/air_log_"+today+".csv"

def air_callback(msg) :
    global air_log
    air_log = msg


    if file_ready_flag and pose_ready_flag :
        f = open(file_name,'a')
        wr = csv.writer(f)

        now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + ":" + str(time.localtime(time.time()).tm_sec)

        log = [now_time]
        log.append(pose_log.position.x)
        log.append(pose_log.position.y)
        for i in range(len(air_log.__getstate__())) :
            log.append(air_log.__getstate__()[i])


        wr.writerow(log)
        f.close()
        print(log)

def pose_callback(msg) :
    global pose_log
    global pose_ready_flag
    pose_log = msg

    pose_ready_flag = True

def main():
    global file_ready_flag

    rospy.init_node("air_log")

    battery_log_save_flag = rospy.get_param("air_log_save_flag",True)

    batt_sub = rospy.Subscriber("/air",EnvironmentMsgs,air_callback)
    pose_sub = rospy.Subscriber("/robot_pose",Pose,pose_callback)


    rospy.sleep(1)




    if os.path.isfile(file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + ['x'] + ['y'] + [i for i in EnvironmentMsgs.__slots__]
        wr.writerow(log_name)
        f.close()
    
    file_ready_flag = True

    rospy.sleep(1)

    rospy.spin()


if __name__ == "__main__" :
    main()
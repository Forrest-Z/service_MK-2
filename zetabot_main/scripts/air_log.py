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

'''
position:
  x: -0.0622398506915
  y: 0.0915844980196
  z: 0.01
orientation:
  x: 0.0
  y: 0.0
  z: 0.999519377801
  w: 0.0310002161404
'''
def air_callback(msg) :
    global air_log
    air_log = msg


    if file_ready_flag and pose_ready_flag :
        f = open(file_name,'a')
        wr = csv.writer(f)

        now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)

        log = [now_time]
        log.append(pose_log.position.x)
        log.append(pose_log.position.y)
        for i in range(len(air_log.__getstate__())) :
            log.append(air_log.__getstate__()[i])


        wr.writerow(log)
        f.close()
        print(log)

'''
Dust_PM2_5_ugm3: 34.0
Dust_PM10_ugm3: 40.0
CO2_ppm: 1039.0
HCHO_ugm3: 45.0
CO_ppm: 0.5
NO2_ppm: 0.0219999998808
Rn_Bqm3: 0.0360708795488
TVOCs_ugm3: 6.9718583893e-38
temp_celcius: 30.0
hum_RHp: 29.6000003815
'''
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
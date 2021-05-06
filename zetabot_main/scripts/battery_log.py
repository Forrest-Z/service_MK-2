#! /usr/bin/env python

import rospy

import pygame
import time
import sys, tty, select, termios, os
from zetabot_main.msg import BatteryInformationMsgs
battery1 = BatteryInformationMsgs()
battery2 = BatteryInformationMsgs()

def battery1_callback(msg) :
    global battery1
    battery1 = msg

def battery2_callback(msg) :
    global battery2
    battery2 = msg



def main():

    rospy.init_node("batt_log")

    batt1_sub = rospy.Subscriber("/battery1",BatteryInformationMsgs,battery1_callback)
    batt1_sub = rospy.Subscriber("/battery2",BatteryInformationMsgs,battery2_callback)
    rospy.sleep(1)
    try :
        while True :
            
            rospy.sleep(1)
            if battery1.voltage >= 25.0 and battery2.voltage >=25.0 :
                """
                if time.localtime(time.time()).tm_min % 30 == 0 :
    
                    log = open('/home/zetabank/batt_log_210506.txt', mode='aw')
                    log.write("\n")
                    log.write("time : " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + "\n")
                    log.write("*"*20 + "\n")
                    log.write("-"*10 + "battery1" + "-"*10 + "\n")
                    log.write(str(battery1) + "\n")
                    log.write("-"*20 + "\n")
                    log.write("-"*10 + "battery2" + "-"*10 + "\n")
                    log.write(str(battery2) + "\n")
                    log.write("*"*20 + "\n")
                    log.write('' + "\n")
                    log.close()
                    print("")
                    print("time : " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + "")
                    print("*"*20 + "")
                    print("-"*10 + "battery1" + "-"*10 + "")
                    print(str(battery1) + "")
                    print("-"*20 + "")
                    print("-"*10 + "battery2" + "-"*10 + "")
                    print(str(battery2) + "")
                    print("*"*20 + "")
                    print('' + "")
                    while time.localtime(time.time()).tm_min % 30 == 0 :
                        rospy.sleep(1)
                """
            else :
                if time.localtime(time.time()).tm_min % 10 == 0 :
    
                    log = open('/home/zetabank/batt_log_210506.txt', mode='aw')
                    log.write("lower 24v\n")
                    log.write("time : " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + "\n")
                    log.write("*"*20 + "\n")
                    log.write("-"*10 + "battery1" + "-"*10 + "\n")
                    log.write(str(battery1) + "\n")
                    log.write("-"*20 + "\n")
                    log.write("-"*10 + "battery2" + "-"*10 + "\n")
                    log.write(str(battery2) + "\n")
                    log.write("*"*20 + "\n")
                    log.write('' + "\n")
                    log.close()
                    print("lower 24v")
                    print("time : " + str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min) + "")
                    print("*"*20 + "")
                    print("-"*10 + "battery1" + "-"*10 + "")
                    print(str(battery1) + "")
                    print("-"*20 + "")
                    print("-"*10 + "battery2" + "-"*10 + "")
                    print(str(battery2) + "")
                    print("*"*20 + "")
                    print('' + "")
                    while time.localtime(time.time()).tm_min % 10 == 0 :
                        rospy.sleep(1)


    except KeyboardInterrupt :
        print("exit")
        sys.exit()

    rospy.spin



        # except Exception:
        #     print("unknown error")
        
    print("Done")

if __name__ == "__main__" :
    main()
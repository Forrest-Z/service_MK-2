
#! /usr/bin/env python

import rospy

import csv
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
    today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

    minit = 99

    rospy.sleep(1)

    file_name = "/home/zetabank/robot_log/battery_log/batt_log_"+today+".csv"


    if os.path.isfile(file_name):
        print('ok')
    else :
        f = open(file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + [j + i for i in BatteryInformationMsgs.__slots__  for j in ['BAT1_','BAT2_']]
        wr.writerow(log_name)
        f.close()

    rospy.sleep(1)


    
    try :
        while True :


            while time.localtime(time.time()).tm_min == minit :
                    rospy.sleep(2)

            f = open(file_name,'a')
            wr = csv.writer(f)
            log_name = ['Time'] + [j + i for i in BatteryInformationMsgs.__slots__  for j in ['BAT1_','BAT2_']]


            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)

            log = [now_time]
            for i in range(len(battery1.__getstate__())) :
                log.append(battery1.__getstate__()[i])
                log.append(battery2.__getstate__()[i])


            wr.writerow(log)
            f.close()
            print(log)

            minit = time.localtime(time.time()).tm_min



    except KeyboardInterrupt :
        print("exit")
        sys.exit()


    rospy.spin()

        # except Exception:
        #     print("unknown error")
        
    print("Done")

if __name__ == "__main__" :
    main()
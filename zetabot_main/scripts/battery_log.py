#! /usr/bin/env python

import rospy

import csv
import time
import sys, tty, select, termios, os
from zetabot_main.msg import BatteryInformationMsgs
battery1 = BatteryInformationMsgs()
battery2 = BatteryInformationMsgs()
log_directory = "/home/zetabank/robot_log/battery_log"
def battery_callback(msg) :
    global battery1
    global battery2
    if msg.id == 0x60 :
        battery1 = msg
    else :
        battery2 = msg


def main():

    rospy.init_node("batt_log")

    batt_sub = rospy.Subscriber("/battery",BatteryInformationMsgs,battery_callback)
    today = time.strftime('%Y_%m_%d', time.localtime(time.time()))

    minit = 99

    rospy.sleep(1)

    file_name = log_directory + "/batt_log_"+today+".csv"


    if os.path.isfile(file_name):
        print('ok')
    else :
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)
        f = open(file_name,'w')
        wr = csv.writer(f)
        log_name = ['Time'] + [j + i for i in BatteryInformationMsgs.__slots__  for j in ['BAT1_','BAT2_']]
        wr.writerow(log_name)
        f.close()

    rospy.sleep(1)


    
    try :
        while (rospy.is_shutdown) :


            while time.localtime(time.time()).tm_min == minit :
                    rospy.sleep(2)

            f = open(file_name,'a')
            wr = csv.writer(f)
            log_name = ['Time'] + [j + i for i in BatteryInformationMsgs.__slots__[1:]  for j in ['BAT1_','BAT2_']]


            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)

            log = [now_time]
            for i in range(len(battery1.__getstate__())) :
                log.append(battery1.__getstate__()[i])
                log.append(battery2.__getstate__()[i])


            wr.writerow(log)
            f.close()
            print(log)

            minit = time.localtime(time.time()).tm_min

        # rospy.spin()

        print("done")



    except:
        print("exit")

        # except Exception:
        #     print("unknown error")
        
    print("Done")


if __name__ == "__main__" :
    main()
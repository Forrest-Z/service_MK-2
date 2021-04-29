#!/usr/bin/env python
import rospy

import os, threading

from std_msgs.msg import Bool, String


sonar_warning_top = 40
sonar_warning_bottom = 35
sonar_stop_top = 30
sonar_stop_bottom = 25

rospy.init_node('sonar_filter')

class SonarSensor :
    def __init__ (self) :
        sonar_UF = 0
        sonar_UR = 0
        sonar_UB = 0
        sonar_UL = 0
        sonar_DF = 0
        sonar_DR1 = 0
        sonar_DR2 = 0
        sonar_DB1 = 0
        sonar_DB2 = 0
        sonar_DL1 = 0
        sonar_DL2 = 0

        # sonar_topic = "/sonar"
        # rospy.Subscriber(sonar_topic,SonarArray,self.recv_sonar)



    def recv_sonar(self,data):
        self.sonar_UF = data.data[5]
        self.sonar_UR = data.data[6]
        self.sonar_UB = data.data[7]
        self.sonar_UL = data.data[10]
        self.sonar_DF = data.data[4]
        self.sonar_DR1 = data.data[1]
        self.sonar_DR2 = data.data[0]
        self.sonar_DB1 = data.data[9]
        self.sonar_DB2 = data.data[8]
        self.sonar_DL1 = data.data[2]
        self.sonar_DL2 = data.data[3]

class RecvEmergency :
    def __init__(self):
        bumper_flag = False
        estop_flag = False

        bumper_topic = "/bumper"
        rospy.Subscriber(bumper_topic,Bool,self.recv_bumper)

        estop_topic = "/estop"
        rospy.Subscriber(estop_topic,Bool,self.recv_estop)

    
    def recv_bumper(self,data):
        self.bumper_flag = data.data

    def recv_estop(self,data):
        self.estop_flag = data.data 


def emergency_send() :

    emergency_topic = "/emergency_stop"
    emergency_pub = rospy.Publisher(emergency_topic,String, queue_size=10)

    sonar = SonarSensor()
    emergency = RecvEmergency()

    while True:
        rospy.sleep(0.01)
        emergency_msg = ""
        os.system("clear")
        # print("sonar_UF :",sonar.sonar_UF)
        # print("sonar_UR :",sonar.sonar_UR)
        # print("sonar_UB :",sonar.sonar_UB)
        # print("sonar_UL :",sonar.sonar_UL)
        # print("sonar_DF :",sonar.sonar_DF)
        # print("sonar_DR1 :",sonar.sonar_DR1)
        # print("sonar_DR2 :",sonar.sonar_DR2)
        # print("sonar_DB1 :",sonar.sonar_DB1)
        # print("sonar_DB2 :",sonar.sonar_DB2)
        # print("sonar_DL1 :",sonar.sonar_DL1)
        # print("sonar_DL2 :",sonar.sonar_DL2)

        if self.bumper_flag == True : 
            emergency_msg += "/bumper_stop"

        if self.estop_flag == True :
            emergency_msg += "/emergency_button_stop"


        # if (sonar.sonar_UF <= sonar_stop_top or sonar.sonar_DF <= sonar_stop_bottom  ) :
        #     emergency_msg += "/sonar_stop"
        #     print("sonar_val :",sonar.sonar_UF)
        #     print("sonar_stop")

        # elif (sonar.sonar_UF <= sonar_warning_top or sonar.sonar_DF <= sonar_warning_bottom  ) :
        #     emergency_msg += "/sonar_warning"
        #     print("sonar_val :",sonar.sonar_UF)
        #     print("sonar_warning")

        emergency_pub.publish(emergency_msg)

def main():
    EsendThd = threading.Thread(target=emergency_send)
    EsendThd.daemon = True 
    EsendThd.start()

    rospy.spin()

if __name__ == "__main__":
    main()
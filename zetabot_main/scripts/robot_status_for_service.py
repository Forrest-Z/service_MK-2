#!/usr/bin/env python
import rospy
import os

from std_msgs.msg import String, UInt8, UInt16, Bool
from zetabot_main.msg import BatteryInformationnMsgs
from zetabot_main.srv import ModuleControllerSrv

class RobotStatus:
    def __init__(self) :
        self.status_data = {
            "battery" = None,
            "emergency" = None,
            "robot_mode" = None,
            "charging" = None,
            "speak" = None
        }

        self.battery_low_margin = 25
        self.battery_midle_margin = 80
        self.battery_full_margin = 97
        self.battery_hysteresis_low = 0
        self.battery_hysteresis_midle = 0
        self.battery_hysteresis_high = 0
        self.battery_hysteresis_full = 0
        self.battery_low_flag = False
        self.battery_midle_flag = False
        self.battery_high_flag = False
        self.battery_full_flag = False

        battery_sub = rospy.Subscriber("/battery",BatteryInformationnMsgs,self.recv_battery)
        emergency_sub = rospy.Subscriber("/emergency_stop",String,self.recv_emergency)
        mode_sub = rospy.Subscriber("/robot_mode",String,self.recv_robot_mode)
        charging_sub = rospy.Subscriber("/charging",Bool,self.recv_charging)
        speak_sub = rospy.Subscriber("/speak",Bool,self.recv_speak)

        self.cur_robot_mode = ''

        self.module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

        self.f_color_ = 0x000000
        self.b_color_ = 0x000000

    def status_led(self):
        os.system("clear")
        hysteresis_term = 3

        try :
            if not("charging" in self.status_data["robot_mode"]) :
                self.battery_hysteresis_low = 0
                self.battery_hysteresis_midle = 0
                self.battery_hysteresis_high = 0

            if self.status_data["robot_mode"] == 'charging':
                self.cur_robot_mode = 'charging'
                f_led_mode = "fade"
                b_led_mode = "fade"
                if self.status_data["battery"] <= self.battery_low_margin - self.battery_hysteresis_low :
                    print("battery_low_charging1111")
                    self.battery_hysteresis_low = hysteresis_term
                    print("1")
                    self.battery_low_flag = True
                    print("2")
                    f_led_color = "red"
                    print("3")
                    b_led_color = "red"
                    
                elif self.battery_low_margin + self.battery_hysteresis_low <= self.status_data["battery"] < self.battery_midle_margin - self.battery_hysteresis_midle :
                    print("battery_midle_charging")
                    self.battery_hysteresis_low = 0
                    self.battery_hysteresis_midle = hysteresis_term
                    self.battery_low_flag = False
                    f_led_color = "orange"
                    b_led_color = "orange"

                elif self.battery_midle_margin + self.battery_hysteresis_midle <= self.status_data["battery"] < self.battery_full_margin - self.battery_hysteresis_high :                
                    print("battery_high_charging")
                    self.battery_hysteresis_midle = 0
                    self.battery_hysteresis_high = hysteresis_term
                    f_led_color = "green"
                    b_led_color = "green"

                elif self.battery_full_margin <= self.status_data["battery"] :
                    print("battery_max_charging")
                    f_led_color = "green"
                    b_led_color = "green"
                    f_led_mode = "on"
                    b_led_mode = "on"
                
                led_command = f_led_mode + "/" + f_led_color "/" + b_led_mode + "/" + b_led_color

            elif self.status_data["robot_mode"] == 'low_battery' :
                print("low_battery")
                self.cur_robot_mode = 'low_battery'
                led_command = "on/red/on/red"

            elif self.status_data["emergency"] != '':
                mode = "blink_fast"

                if 'stop' in self.status_data["emergency"] :
                    print("emergency_stop")
                    color = "red"

                elif 'warning' in self.status_data["emergency"]:
                    print("emergency_warning")
                    color = "orange"
                
                led_command = mode + "/" + color "/" + mode + "/" + color

            elif self.status_data["robot_mode"] == 'service' :
                print("service")
                self.cur_robot_mode = 'service'
                led_command = "on/yellow/on/white"

            elif self.status_data["robot_mode"] == "QR" :
                print("QR_code")
                self.cur_robot_mode = 'service'
                led_command = "sweep/blue/stay/stay"
            
            elif self.status_data["robot_mode"] == "face_detect" :
                print("face_detect")
                self.cur_robot_mode = 'service'
                led_command = "sweep/green/stay/stay"

            elif self.status_data["robot_mode"] == 'normal' :
                print("normal")
                self.cur_robot_mode = 'normal'
                led_command = "on/white/on/white"

            elif self.status_data["robot_mode"] == 'full_coverage' :
                print("full_coverage")
                self.cur_robot_mode = 'full_coverage'
                led_command = "on/blue/on/blue"

            elif self.status_data["robot_mode"] == 'air_condition' :
                print("air_condition")
                self.cur_robot_mode = 'air_condition'
                led_command = "on/sky/on/sky"

            else :
                print("None")
                led_command = "on/white/on/white"

            print("ssss")

            if self.status_data["speak"] :
                f_led_mode = Led.Mode.sweep
                led_command = "sweep/stay/stay/stay"
                if self.cur_robot_mode == 'normal' :
                    led_command = "sweep/yellow/stay/stay"

            self.module_controller_srv("led/"+led_command)
        except :
            pass

    def recv_battery(self,msg) :
        if self.status_data["battery"] == None or self.status_data["battery"] != msg.remaining_capacity :
            self.status_data["battery"] = msg.remaining_capacity
            self.status_led()
        
    def recv_emergency(self,msg) :
        if self.status_data["emergency"] == None or self.status_data["emergency"] != msg.data :
            self.status_data["emergency"] = msg.data
            self.status_led()

    def recv_robot_mode(self,msg) :
        if self.status_data["robot_mode"] == None or self.status_data["robot_mode"] != msg.data :
            self.status_data["robot_mode"] = msg.data
            self.status_led()            

    def recv_charging(self,msg) :
        if self.status_data["charging"] == None or self.status_data["charging"] != msg.data :
            self.status_data["charging"] = msg.data
            self.status_led()            

    def recv_speak(self,msg) :
        if self.status_data["speak"] == None or self.status_data["speak"] != msg.data :
            self.status_data["speak"] = msg.data
            self.status_led()            



def main() :
    rospy.init_node("robot_status")

    battery_status = RobotStatus()


    rospy.spin()    

if __name__ == "__main__":
    main()
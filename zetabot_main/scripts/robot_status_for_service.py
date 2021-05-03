#!/usr/bin/env python
import rospy
import os

from std_msgs.msg import String
from std_msgs.msg import UInt8,UInt16, UInt64
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from zetabot_main.srv import ModuleControllerSrv


class RosMaker(object):
    def __init__(self,type,topic_name,message_type):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,self.callback)
        
        # battery_level_topic = "/battery_level"
        # battery_level_pub = rospy.Publisher(battery_level_topic,UInt8,queue_size=10)
 
    # instance method
    def callback(self,_msgs):
        self.msg = _msgs

    def publish(self,msgs):
        self._pub.publish(msgs)

class Led :
    class Color:
        stay = 0x000000
        white = 0xffffff
        blue = 0x0000ff
        sky = 0x00ffff
        green = 0x00ff00
        yellow = 0xffff00
        red = 0xff0000
        orange = 0xff7800

    # class Color:
    #     stay = 0x000000
    #     white = 0xFFFFFF
    #     blue = 0x0000FF
    #     sky = 0x00FFFF
    #     green = 0x00FF00
    #     yellow = 0xFFFF00
    #     red = 0xE51400
    #     orange = 0xF0A30B

    class Mode:
        off = 0x0
        on = 0x1
        blink = 0x2
        blink_fast = 0x3
        fade = 0x4
        sweep = 0x5
        sweep_fast = 0x6
        stay = 0xff
 

class RobotStatus:
    battery_low_margin = 25
    battery_midle_margin = 80
    battery_full_margin = 97
    battery_hysteresis_low = 0
    battery_hysteresis_midle = 0
    battery_hysteresis_high = 0
    battery_hysteresis_full = 0
    battery_low_flag = False
    battery_midle_flag = False
    battery_high_flag = False
    battery_full_flag = False

    def __init__(self):
        self.battery_sub = RosMaker("sub","/battery",UInt16)
        self.emergency_sub = RosMaker("sub","/emergency_stop",String)
        self.mode_sub = RosMaker("sub","/robot_mode",String)
        self.charging_sub = RosMaker("sub","/charging",Bool)
        self.speak_sub = RosMaker("sub","/speak",Bool)

        self.battery_level_pub = RosMaker("pub","/battery_level",UInt8)
        self.led_command_pub = RosMaker("pub","/led_command",UInt64)
        
        self.battery_set_flag = False
        self.led_count = 1000
        self.led_color = "blue"
        self.blink_flag = False
        self.warring_flag = False
        self.stop_flag = False
        self.charging_flag = False
        self.mode_led_flag = ""

    def status_led(self):

        #1.charging
        #2.ultra_sonic
        #3.battery
        #4.mode

        try:
            emergency_msg = self.emergency_sub.msg.data
            robot_mode_msg = self.mode_sub.msg.data
            battery_level = self.battery_level
            charging_msg = self.charging_sub.msg.data
            hysteresis_term = 3

            os.system("clear")

            print("ready",self.battery_level)

            if emergency_msg == "" :
                self.warring_flag = False
                self.stop_flag = False
            if robot_mode_msg == "" :
                self.mode_led_flag = ""
            if charging_msg == False :
                self.charging_flag  = False

            print("ready",battery_level)
            print("charging_msg", charging_msg)
            print("emergency_msg", emergency_msg)
            print("battery_level", battery_level)
            print("robot_mode_msg", robot_mode_msg)

            # if battery_level <= self.battery_low_margin-self.battery_hysteresis :
            #     battery_low_flag = True
            # elif battery_level >= self.battery_low_margin+self.battery_hysteresis :
            #     battery_low_flag = False                

            # if "charging" in robot_mode_msg : 
            #     if battery_low_flag :
            #         print("battery_low_charging")
            #         self.led_command_pub.publish(1)
            #         print("battery_low_charging")
            #     elif self.battery_low_margin + self.battery_hysteresis <= battery_level < self.battery_midle_margin - self.battery_hysteresis :
            #         self.led_command_pub.publish(2)
            #         print("battery_midle_charging")
            #     elif self.battery_midle_margin + self.battery_hysteresis <= battery_level < self.battery_full_margin - self.battery_hysteresis :
            #         self.led_command_pub.publish(3)
            #         print("battery_high_charging")
            #     elif self.battery_full_margin <= battery_level :
            #         self.led_command_pub.publish(4)
            #         print("battery_max_charging")

            if not("charging" in robot_mode_msg) :
                self.battery_hysteresis_low = 0
                self.battery_hysteresis_midle = 0
                self.battery_hysteresis_high = 0

            if "charging" in robot_mode_msg :
                if battery_level <= self.battery_low_margin - self.battery_hysteresis_low :
                    print("battery_low_charging")
                    self.led_command_pub.publish(1)
                    self.battery_hysteresis_low = hysteresis_term
                    self.battery_low_flag = True
                elif self.battery_low_margin + self.battery_hysteresis_low <= battery_level < self.battery_midle_margin - self.battery_hysteresis_midle :
                    print("battery_midle_charging")
                    self.led_command_pub.publish(2)
                    self.battery_hysteresis_low = 0
                    self.battery_hysteresis_midle = hysteresis_term
                    self.battery_low_flag = False
                elif self.battery_midle_margin + self.battery_hysteresis_midle <= battery_level < self.battery_full_margin - self.battery_hysteresis_high :                
                    print("battery_high_charging")
                    self.led_command_pub.publish(3)
                    self.battery_hysteresis_midle = 0
                    self.battery_hysteresis_high = hysteresis_term
                elif self.battery_full_margin <= battery_level :
                    print("battery_max_charging")
                    self.led_command_pub.publish(4)


            elif emergency_msg != "" and robot_mode_msg != "charge_search" :
                print("emergency_msg")
                if "stop" in emergency_msg :
                    self.led_command_pub.publish(5)
                    print("robot_stop")
                elif "warning" in emergency_msg :
                    self.led_command_pub.publish(6)
                    print("robot_warning")

            elif self.battery_low_flag:
                self.led_command_pub.publish(7)
                print("battery_low")
            
            elif robot_mode_msg != "" :
                if "full_coverage" in robot_mode_msg :
                    self.led_command_pub.publish(8)
                    print("full_coverage_mode")
                elif "air_condition" in robot_mode_msg :
                    self.led_command_pub.publish(9)
                    print("air_conditioning_mode")

        except:
            pass

        

    def battery_level_publisher(self):
        battery_min_margin = 200
        battery_max_margin = 900

        if self.battery_sub.msg.data != None:
            print(self.battery_sub.msg.data)
            self.battery_level = int(float(float(self.battery_sub.msg.data-battery_min_margin) / battery_max_margin) * 100)
            # print("battery_level_data : ",self.battery_level)
            print(self.battery_level)
            self.battery_level_pub.publish(self.battery_level)
            

class led_controller(RobotStatus) :
    def __init__(self) :
        RobotStatus.__init__(self)
        self.led_pub = RosMaker("pub","/led_command",UInt64)

        self.cur_robot_mode = ''

        self.f_color_ = 0x000000
        self.b_color_ = 0x000000

        
        

    def led_custom(self,f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue) :
        command = 0
        for i in [f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue] :
            print(command)
            print(i)
            print(type(i))
            command = (command << 8) | i
        self.led_pub.publish(command)

    def led_controll(self,f_mode,b_mode,f_color,b_color) :
        if f_color == 0x000000 :
            f_color = self.f_color_
        if b_color == 0x000000 :
            b_color = self.b_color_

        self.f_color_, self.b_color_ = f_color, b_color
        command = ((f_mode << 24) | f_color) << 32 | ((b_mode << 24) | b_color)
        self.led_pub.publish(command)
        print(hex(command))
        

    def status_led(self):

        os.system("clear")
        hysteresis_term = 3

        try :
            if not("charging" in self.mode_sub.msg.data) :
                    self.battery_hysteresis_low = 0
                    self.battery_hysteresis_midle = 0
                    self.battery_hysteresis_high = 0

            if self.mode_sub.msg.data == 'charging':
                self.cur_robot_mode = 'charging'
                f_led_mode = Led.Mode.fade
                b_led_mode = Led.Mode.fade
                if self.battery_level <= self.battery_low_margin - self.battery_hysteresis_low :
                    print("battery_low_charging1111")
                    self.battery_hysteresis_low = hysteresis_term
                    print("1")
                    self.battery_low_flag = True
                    print("2")
                    f_led_color = Led.Color.red
                    print("3")
                    b_led_color = Led.Color.red
                    
                elif self.battery_low_margin + self.battery_hysteresis_low <= self.battery_level < self.battery_midle_margin - self.battery_hysteresis_midle :
                    print("battery_midle_charging")
                    self.battery_hysteresis_low = 0
                    self.battery_hysteresis_midle = hysteresis_term
                    self.battery_low_flag = False
                    f_led_color = Led.Color.orange
                    b_led_color = Led.Color.orange

                elif self.battery_midle_margin + self.battery_hysteresis_midle <= self.battery_level < self.battery_full_margin - self.battery_hysteresis_high :                
                    print("battery_high_charging")
                    self.battery_hysteresis_midle = 0
                    self.battery_hysteresis_high = hysteresis_term
                    f_led_color = Led.Color.green
                    b_led_color = Led.Color.green

                elif self.battery_full_margin <= self.battery_level :
                    print("battery_max_charging")
                    f_led_color = Led.Color.green
                    b_led_color = Led.Color.green
                    f_led_mode = Led.Mode.stay
                    b_led_mode = Led.Mode.stay

            elif self.emergency_sub.msg.data != '' and self.cur_robot_mode != 'service':
                f_led_mode = Led.Mode.blink_fast
                b_led_mode = Led.Mode.blink_fast

                if 'stop' in self.emergency_sub.msg.data :
                    print("emergency_stop")
                    f_led_color = Led.Color.red
                    b_led_color = Led.Color.red

                elif 'warning' in self.emergency_sub.msg.data and self.cur_robot_mode != 'low_battery' :
                    print("emergency_warning")
                    f_led_color = Led.Color.orange
                    b_led_color = Led.Color.orange
                
            elif self.mode_sub.msg.data == 'low_battery' :
                print("low_battery")
                self.cur_robot_mode = 'low_battery'
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.red
                b_led_color = Led.Color.red

            elif self.mode_sub.msg.data == 'service' :
                print("service")
                self.cur_robot_mode = 'service'
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.yellow
                b_led_color = Led.Color.white

            elif self.mode_sub.msg.data == "QR" :
                print("QR_code")
                self.cur_robot_mode = 'service'
                f_led_mode = Led.Mode.sweep
                f_led_color = Led.Color.blue
                b_led_mode = Led.Mode.stay
                b_led_color = Led.Color.stay
            
            elif self.mode_sub.msg.data == "face_detect" :
                print("face_detect")
                self.cur_robot_mode = 'service'
                f_led_mode = Led.Mode.sweep
                f_led_color = Led.Color.green
                b_led_mode = Led.Mode.stay
                b_led_color = Led.Color.stay

            elif self.mode_sub.msg.data == 'normal' :
                print("normal")
                self.cur_robot_mode = 'normal'
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.white
                b_led_color = Led.Color.white

            elif self.mode_sub.msg.data == 'full_coverage' :
                print("full_coverage")
                self.cur_robot_mode = 'full_coverage'
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.blue
                b_led_color = Led.Color.blue

            elif self.mode_sub.msg.data == 'air_condition' :
                print("air_condition")
                self.cur_robot_mode = 'air_condition'
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.sky
                b_led_color = Led.Color.sky

            else :
                print("None")
                f_led_mode = Led.Mode.on
                b_led_mode = Led.Mode.on
                f_led_color = Led.Color.white
                b_led_color = Led.Color.white

            print("ssss")

            if self.speak_sub.msg.data :
                f_led_mode = Led.Mode.sweep
                if self.cur_robot_mode == 'normal' :
                    f_led_color = Led.Color.yellow

            print("speak_flag : ", self.speak_sub.msg.data)
            print("robot_mode : ", self.mode_sub.msg.data)
            print("cur_robot_mode : ", self.cur_robot_mode)
            print("emergency_sub : ", self.emergency_sub.msg.data)
            print("battery_level : ", self.battery_level)
            

            self.led_controll(f_led_mode,b_led_mode,f_led_color,b_led_color)
        except :
            pass
        


rospy.init_node("robot_status")

battery_status = led_controller()
# battery_level_pub = battery_status.battery_level_publisher()


while(rospy.is_shutdown) :
    battery_status.battery_level_publisher()
    battery_status.status_led()
    # print(battery_level)
    rospy.sleep(0.5)

rospy.spin()  


# def battery_level_status(battery_level):
    

# def battery_recv(msg) :
#     battery_data = int(msg.data)
#     battery_level = int(((battery_data-340)/1100) * 100)
#     battery_level_status(battery_level)


# def main() :
#     rospy.init_node("robot_status")

#     battery_toppic = "/battery"
#     rospy.Subscriber("/battery",String,battery_recv)



#     rospy.spin()    

# if __name__ == "__main__":
#     main()
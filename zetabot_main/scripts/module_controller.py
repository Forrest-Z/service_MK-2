#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import UInt16, Bool, UInt64

from zetabot_main.srv import ModuleControllerSrv
from zetabot_main.msg import ModuleControlMsgs
from threading import Thread

#-------------------------------------------------
# from zetabot_main.srv import ModuleControllerSrv

# module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)


# module_controller_srv("uvc_on,led_off,led_green,air_lv2,air_off")
#-------------------------------------------------




class Led :
    Color = {
        "off" : 0x000000,
        "white" : 0xffffff,
        "blue" : 0x0000ff,
        "sky" : 0x00ffff,
        "green" : 0x00ff00,
        "yellow" : 0xffff00,
        "red" : 0xff0000,
        "orange" : 0xff7800
    }

    # class Color:
    #     stay = 0x000000
    #     white = 0xFFFFFF
    #     blue = 0x0000FF
    #     sky = 0x00FFFF
    #     green = 0x00FF00
    #     yellow = 0xFFFF00
    #     red = 0xE51400
    #     orange = 0xF0A30B

    Mode = {
        "off" : 0x0,
        "on" : 0x1,
        "blink" : 0x2,
        "blink_fast" : 0x3,
        "fade" : 0x4,
        "sweep" : 0x5,
        "sweep_fast" : 0x6,
        "stay" : 0xff
    }

class LedControl :
    def __init__() :
        self.f_color_ = 0x000000
        self.b_color_ = 0x000000

        led_command_topic = "/led_command"
        self.led_command_pub = rospy.Publisher(led_command_topic,UInt64,queue_size=10)

 
    def led_control(self,f_mode,f_color,b_mode,b_color) :
        if f_color == "stay" :
            f_color = self.f_color_
        if b_color == "stay" :
            b_color = self.b_color_

        self.f_color_, self.b_color_ = Led.Color[f_color], Led.Color[b_color]
        command = ((Led.Color[f_mode] << 24) | Led.Color[f_color]) << 32 | ((Led.Color[b_mode] << 24) | Led.Color[b_color])
        self.led_command_pub.publish(command)
        print(hex(command))

    def led_custom(self,f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue) :
        command = 0
        for i in [f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue] :
            command = (command << 8) | i
        self.led_command_pub.publish(command)
 


class ModuleController :
    def __init__(self):
        self.pulifier_level = {
            "off" : 0,
            "lv1" : 100,
            "lv2" : 250,
            "lv3" : 400
        }

        self.purifier_command = UInt16()
        self.uvc_command = Bool()
        self.pump_command = Bool()

        purifier_topic = "/purifier_command"
        self.purifier_pub = rospy.Publisher(purifier_topic,UInt16,queue_size=10)

        uvc_topic = "/uvc"
        self.uvc_pub = rospy.Publisher(uvc_topic,Bool,queue_size=10)

        pump_topic = "/pump"
        self.pump_pub = rospy.Publisher(pump_topic,Bool,queue_size=10)


        self.led_controller = LedControl()
        self.led_controller.led_control("off","off","off","off")
        rospy.sleep(0.1)

        self.purifier_pub.publish(self.purifier_command)
        rospy.sleep(0.1)
        self.uvc_pub.publish(self.uvc_command)
        rospy.sleep(0.1)
        self.pump_pub.publish(self.pump_command)
        rospy.sleep(0.1)


        module_control_srv = rospy.Service('/module_controller_srv', ModuleControllerSrv, self.module_controller)



    def module_controller(self,comm_list):
        
        comm_list = comm_list.command.split(",")

        for i in comm_list :
            
            print(i)
            if "led" in i  :
                comm = i.split("/")
                self.led_controller.led_control(comm[1],comm[2],comm[3],comm[4])

            elif "air"  in i :
                comm = i.split("_")
                print("air")
                self.purifier_command.data = self.pulifier_level[comm[1]]
                self.purifier_pub.publish(self.purifier_command)

            elif "uvc"  in i :
                if "_on" in i :
                    self.uvc_command.data = True
                elif "_off" in i :
                    self.uvc_command.data = False
                    
                self.uvc_pub.publish(self.uvc_command)
            
            elif "pump"  in i :
                if "_on" in i :
                    self.pump_command.data = True
                elif "_off" in i :
                    self.pump_command.data = False
                    
                self.pump_pub.publish(self.pump_command)

                    
def main() :
    rospy.init_node("module_controller_server")

    rospy.sleep(1)

    srv = ModuleController()

    print("Module_control Ready")
    print("-"*20)

    rospy.spin()

if __name__ == "__main__":
    main()

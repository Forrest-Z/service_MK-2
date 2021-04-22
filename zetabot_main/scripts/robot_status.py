#!/usr/bin/env python
import rospy
import os

from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt64
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
 
    # staticmethod
    @staticmethod
    def isSquare(rectWidth, rectHeight):
        return rectWidth == rectHeight   
 
class led_controller(object) :
    # Uint64_t -> 0x[2n][6n][2n][6n]     [전면cmd][전면RGB][후면cmd][후면rgb]


    # enum class LED_CMD : int8_t{
    #     kCMD_OFF,
    #     kCMD_ON,
    #     kCMD_BLINK,
    #     kCMD_BLINK_FAST,
    #     KCMD_FADE,
    #     kCMD_SWEEP,
    #     kCMD_SWEEP_FAST,
    #     kCMD_FOO,
    # };

    def __init__(self,node_name) :
        self.led_pub = RosMaker("pub","/led_command",UInt64)

    def led_controll(self,f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue) :
        command = 0
        for i in [f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue] :
            command = (command << 8) | i
        self.led_pub.publish(command)

    def led_mode(self,mode) :


class RobotStatus():
    battery_low_margin = 20
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
        self.battery_sub = RosMaker("sub","/battery",String)
        self.emergency_sub = RosMaker("sub","/emergency_stop",String)
        self.mode_sub = RosMaker("sub","/robot_mode",String)
        self.charging_sub = RosMaker("sub","/charging",Bool)

        self.battery_level_pub = RosMaker("pub","/battery_level",UInt8)
        self.led_command_pub = RosMaker("pub","/led_command",Int64)

        self.led_controller = led_controller()

        self.module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)
        
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
                
            # if battery_level <= 20 and (not(self.battery_set_flag) or (battery_level != self.battery_level_temp)) :
            #     print("blink")
            #     self.module_controller_srv("led_red_"+str(battery_level)+",led_blink_1_on")
            #     self.battery_level_temp = battery_level
            #     print(self.battery_level_temp != battery_level)
            #     self.battery_set_flag = True

        except:
            pass

        

    def battery_level_publisher(self):
        battery_min_margin = 350
        battery_max_margin = 900

        if self.battery_sub.msg.data != '':
            self.battery_level = int(float(float(int(self.battery_sub.msg.data)-battery_min_margin) / battery_max_margin) * 100)
            # print("battery_level_data : ",self.battery_level)
            self.battery_level_pub.publish(self.battery_level)
            


        
        


rospy.init_node("robot_status")

battery_status = RobotStatus()
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
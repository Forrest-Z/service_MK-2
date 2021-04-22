#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from std_msgs.msg import UInt8
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
 
    # classmethod
    @classmethod
    def printCount(cls):
        print(cls.count)   
 
class RobotStatus():
    

    def __init__(self):
        self.battery_sub = RosMaker("sub","/battery",String)
        self.emergency_sub = RosMaker("sub","/emergency_stop",String)
        self.mode_sub = RosMaker("sub","/robot_mode",String)
        self.charging_sub = RosMaker("sub","/charging",Bool)

        self.battery_level_pub = RosMaker("pub","/battery_level",UInt8)

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
            
            print("ready",self.battery_level)

            if emergency_msg == "" :
                self.warring_flag = False
                self.stop_flag = False
            if robot_mode_msg == "" :
                self.mode_led_flag = ""
            if charging_msg == False :
                self.charging_flag  = False
        
            if charging_msg == True :
                print("charging_msg")
                self.charging_led()                

            elif emergency_msg != "" :
                print("emergency_msg")
                self.emergency_led()

            elif battery_level <= 20 :
                print("battery_level")
                self.low_battery_led()
            
            elif robot_mode_msg != "" :
                print("robot_mode_msg")
                self.robot_mode_led()
                
            # if battery_level <= 20 and (not(self.battery_set_flag) or (battery_level != self.battery_level_temp)) :
            #     print("blink")
            #     self.module_controller_srv("led_red_"+str(battery_level)+",led_blink_1_on")
            #     self.battery_level_temp = battery_level
            #     print(self.battery_level_temp != battery_level)
            #     self.battery_set_flag = True

        except:
            pass
    
    def charging_led(self):
        if self.blink_flag == False :
            self.module_controller_srv("led_blink_on_1")
            self.blink_flag = True

        if self.charging_flag == False :
            if self.battery_level <= 20:
                print("charging and battery <= 20")
                self.module_controller_srv("led_orange_"+str(self.battery_level)+"")
            else :
                self.module_controller_srv("led_green_"+str(self.battery_level)+"")
            self.led_charging_count = self.battery_level
            self.charging_flag = True

        if self.led_charging_count != self.battery_level :
            if self.battery_level <= 20:
                print("charging and battery <= 20")
                self.module_controller_srv("led_orange_"+str(self.battery_level)+"")
            else :
                self.module_controller_srv("led_green_"+str(self.battery_level)+"")
            self.led_charging_count = self.battery_level

    def emergency_led(self):
        self.led_count = 100
        self.mode_led_flag = ""

        if "warring" in self.emergency_sub.msg.data and self.warring_flag == False :
            self.module_controller_srv("led_"+self.led_color+"_"+str(self.led_count)+",led_blink_on_1")
            print("warring")
            self.warring_flag = True
            self.stop_flag = False
    
        elif "stop" in self.emergency_sub.msg.data and self.stop_flag == False :
            self.module_controller_srv("led_"+self.led_color+"_"+str(self.led_count)+",led_blink_on_0.5")
            print("stop")
            self.warring_flag = False
            self.stop_flag = True


    def low_battery_led(self):
        if self.blink_flag == False :
            self.module_controller_srv("led_blink_on_1")
            print("led_blink_on_1")
            self.blink_flag = True

        if self.led_count != self.battery_level :
            self.module_controller_srv("led_red_"+str(self.battery_level)+"")
            print("low_battery")
            self.led_count = self.battery_level


    def robot_mode_led(self):
        if self.blink_flag == False :
            self.blink_flag = True
        if self.mode_sub.msg.data == "air" and self.mode_led_flag != "air" :
            self.led_color = "blue"
            self.module_controller_srv("led_"+self.led_color+"_spin")
            print("air_mode")
            self.mode_led_flag = "air"

        elif self.mode_sub.msg.data == "full" and self.mode_led_flag != "full":
            self.led_color = "green"
            self.module_controller_srv("led_"+self.led_color+"_spin")
            print("full_mode")
            self.mode_led_flag = "full"

        

    def battery_level_publisher(self):
        if self.battery_sub.msg.data != '':
            self.battery_level = int(float(float(int(self.battery_sub.msg.data)-330) / 1100) * 100)
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
#!/usr/bin/env python
import rospy

import sys, select, termios, tty

from std_msgs.msg import UInt64

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

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

class led_controller(object) :
    def __init__(self,node_name) :
        rospy.init_node(node_name)
        self.led_pub = RosMaker("pub","/led_command",UInt64)

    def led_controll(self,f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue) :
        command = 0
        print("ffff",f_mode)
        for i in [f_mode,f_red,f_green,f_blue,b_mode,b_red,b_green,b_blue] :
            print(command)
            print(i)
            print(type(i))
            command = (command << 8) | i
        self.led_pub.publish(command)
        print(hex(command))

led_con = led_controller("test")

rospy.sleep(1)


while(not rospy.is_shutdown()):

    # mode = int(input("led_mode : "))

    mode = 0x03

    print(mode)
    print(type(mode))

    # led_con.led_controll(mode,0xff,0x00,0x00,mode,0xff,0x00,0x00)

    # rospy.sleep(1)

    # led_con.led_controll(mode,0x00,0xff,0x00,mode,0x00,0xff,0x00)

    rospy.sleep(0.5)

    led_con.led_controll(mode,0x00,0x00,0xff,mode,0x00,0x00,0xff)
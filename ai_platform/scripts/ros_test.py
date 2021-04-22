#! /usr/bin/env python

import rospy

import time
import sys, tty, select, termios

from darknet_ros_msgs.msg import BoundingBoxesCenter



class RosMaker(object):
    def __init__(self,type,topic_name,message_type,callback = None):
        self._topic_name = topic_name
        self._message_type = message_type
        if type == "pub" :
            self._pub = rospy.Publisher(self._topic_name,self._message_type,queue_size=10)

        elif type == "sub" :
            if callback == None :
                callback = self.callback
            self.msg = message_type()
            self._sub = rospy.Subscriber(self._topic_name,self._message_type,callback)
        
        # battery_level_topic = "/battery_level"
        # battery_level_pub = rospy.Publisher(battery_level_topic,UInt8,queue_size=10)
 
    # instance method
    def callback(self,_msgs):
        self.msg = _msgs

    def publish(self,msgs):
        self._pub.publish(msgs)


class SaySomething(object) :
    def __init__(self) :

        self.depth_sub = RosMaker("sub","darknet_depth",BoundingBoxesCenter,self.depthSubCallback)

    def depthSubCallback(self,msg) :
        print(msg)


def main():
    rospy.init_node("say_something")
    
    say = SaySomething()

    rospy.spin()



        # except Exception:
        #     print("unknown error")
        
    print("Done")

if __name__ == "__main__" :
    main()
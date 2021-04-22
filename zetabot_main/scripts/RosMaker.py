#!/usr/bin/env python
import rospy

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
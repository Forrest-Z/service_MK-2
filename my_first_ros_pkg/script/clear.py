#! /usr/bin/env python
import rospy
import rosservice
from std_srvs.srv import Empty, EmptyRequest

if __name__ == '__main__':
    rospy.sleep(4)
    rosservice.call_service('/move_base/clear_costmaps',Empty)
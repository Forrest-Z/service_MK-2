#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import threading

import sys, select, termios, tty

import time

msg = """
Control Your Zetabot!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop
CTRL-C to quit
"""
incdec_val = 0.03

roming_flag = False

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def roming_thd_call_back():
    global roming_flag
    roming_vel = 0.3
    during = 5
    rate = 0.02

    while True :
        while roming_flag :
            now = time.time()
            control_linear_vel  = -1 * roming_vel
            roming_vel = control_linear_vel
            control_angular_vel = 0
            twist = Twist()
            twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel
            while time.time() - now <= during :
                pub.publish(twist)
                rospy.sleep(rate)
        rospy.sleep(0.1)

if __name__=="__main__":
    global roming_flag
    settings = termios.tcgetattr(sys.stdin)
    


    rospy.init_node('zetabank_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    teleop_pub = rospy.Publisher('/teleop', Bool, queue_size=5)

    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0

    roming_thd = threading.Thread(target=roming_thd_call_back)
    roming_thd.daemon = True 
    roming_thd.start()

    try :
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = target_linear_vel + incdec_val
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'j' :
                twist = Twist()
                twist.linear.x = 0.1; twist.linear.y = 0; twist.linear.z = 0
                pub.publish(twist)
                rospy.sleep(10)
                target_linear_vel   = 0
            elif key == 'x' :
                target_linear_vel = target_linear_vel - incdec_val
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'a' :
                target_angular_vel = target_angular_vel + incdec_val
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == 'd' :
                target_angular_vel = target_angular_vel - incdec_val
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0
                control_linear_vel  = 0
                target_angular_vel  = 0
                control_angular_vel = 0
                print vels(0, 0)
            elif key == 'g' :
                status = status + 1
                control_linear_vel  = 0.1
                control_angular_vel = 0
                twist = Twist()
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel
                pub.publish(twist)
                print vels(target_linear_vel,target_angular_vel)
                print 'before sleep'	
                rospy.sleep(20.0)
                print 'after sleep'	
                target_linear_vel   = 0.0

            elif key == 'r' :
                roming_flag = not(roming_flag)

            elif status == 14 :
                print msg
                status = 0
            else:
                if (key == '\x03'):
                    break


            if target_linear_vel > control_linear_vel:
                control_linear_vel = min( target_linear_vel, control_linear_vel + (0.1/4.0) )
            else:
                control_linear_vel = target_linear_vel

            if target_angular_vel > control_angular_vel:
                control_angular_vel = min( target_angular_vel, control_angular_vel + (0.1/4.0) )
            else:
                control_angular_vel = target_angular_vel


            if not roming_flag :
                twist = Twist()
                twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel

                teleop_bool = Bool()
                teleop_bool.data = 1

                pub.publish(twist)
                teleop_pub.publish(teleop_bool)

    except :
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        teleop_bool = Bool()
        teleop_bool.data = 1

        pub.publish(twist)
        teleop_pub.publish(teleop_bool)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

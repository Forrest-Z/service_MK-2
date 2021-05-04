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
from std_msgs.msg import Bool, UInt64
from zetabot_main.srv import ModuleControllerSrv

import sys, select, termios, tty

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
number(1~9) : led_mode
i/k : increase/decrease pulifire level
p : pump on/off
CTRL-C to quit
"""
incdec_val = 0.03

f_color_ = 0x000000
b_color_ = 0x000000


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

def status_led(num):
    module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

    if num == 1:
        comm = "/blink_fast/red/blick_fast/red"

        
    elif num == 2:
        print("low_battery")
        comm = "/on/red/on/red"


    elif num == 3:
        print("service")
        comm = "/on/yellow/on/white"

    elif num == 4:
        print("QR_code")
        comm = "/sweep/blue/stay/stay"
    
    elif num == 5:
        print("face_detect")
        comm = "/sweep/green/stay/stay"


    elif num == 6:
        print("normal")
        comm = "/on/white/on/white"


    elif num == 7:
        print("full_coverage")
        comm = "/on/blue/on/blue"

    elif num == 8:
        print("air_condition")
        comm = "/on/sky/on/sky"

    elif num == 9 :
        print("speak")
        comm = "/sweep/yellow/stay/stay"
    
    module_controller_srv("led"+comm)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('zetabank_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    teleop_pub = rospy.Publisher('/teleop', Bool, queue_size=5)
    module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)
    

    status = 0
    target_linear_vel = 0
    target_angular_vel = 0
    control_linear_vel = 0
    control_angular_vel = 0
    pump_power = False
    pulifier_level = 0
    pulifier_command = 'air_lv1'

    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = target_linear_vel + incdec_val
                status = status + 1
                print vels(target_linear_vel,target_angular_vel)
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

            # elif key == 'p' :
            #     status = status + 1
            #     pump_power = not(pump_power)
            #     if pump_power :
            #         print 'pump_on'
            #         module_controller_srv("pump_on")
            #     else :
            #         print 'pump_off'
            #         module_controller_srv("pump_off")
            
            elif key == 'u' :
                status = status + 1
                pump_power = not(pump_power)
                if pump_power :
                    print 'uvc_on'
                    module_controller_srv("uvc_on")
                else :
                    print 'uvc_off'
                    module_controller_srv("uvc_off")

            elif key in list(map(str,range(1,10))):
                status = status + 1
                print 'led_mode_' + str(key)
                status_led(int(key))

            elif key == 'i' or key == 'k' :
                status = status + 1
                if key == 'i' :
                    pulifier_level += 1
                    pulifier_level = min(pulifier_level,3)
                elif key == 'k' :
                    pulifier_level -= 1
                    pulifier_level = max(pulifier_level,0)
                if pulifier_level == 0 :
                    print 'air_off'
                    module_controller_srv('air_off')
                    continue

                pulifier_command = 'air_lv' + str(pulifier_level)
                print pulifier_command
                module_controller_srv(pulifier_command)

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

            twist = Twist()
            twist.linear.x = control_linear_vel; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_angular_vel

            teleop_bool = Bool()
            teleop_bool.data = 1

            pub.publish(twist)
            teleop_pub.publish(teleop_bool)

    except:
        print "e"

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        teleop_bool = Bool()
        teleop_bool.data = 1

        pub.publish(twist)
        teleop_pub.publish(teleop_bool)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

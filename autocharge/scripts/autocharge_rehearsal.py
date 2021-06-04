#! /usr/bin/env python

import os
import sys
from time import time, sleep
import socket
from threading import Thread
import json
import cv2
from recognition import recongnition
from collections import Counter
import math
import cgitb
cgitb.enable(format= 'text')

#ros rib
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import UInt64
from std_msgs.msg import Char
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

#ros simple goal lib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from zetabot_main.msg import MoveMsgs, MoveBaseActAction
from zetabot_main.srv import TurnSrv
from zetabot_main.msg import ChargingAction,ChargingActionGoal,ChargingFeedback,ChargingActionResult
#from zetabot_main.msg import BatteryInformationMsgs

from zetabot_main.srv import ModuleControllerSrv # hong

##################hong
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
##################hong


class RosFunction:
    cancel_flag = False
    battery_amount = 0.0
    #battery_amount2 = 0
    degree = 0
    station_state = ""

    def __init__(self):
        # publisher setup - move_vel
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.autocharge_publisher = rospy.Publisher('autocharge_state_NUC', UInt8, queue_size = 1)

        # subscriber setup
        battery_amount_subscriber = rospy.Subscriber('/battery_SOC', Float32, self._battery_amount_subscriber_callback)
        #battery2_amount_subscriber = rospy.Subscriber('/battery2', String, self._battery2_amount_subscriber_callback)
        station_subscriber = rospy.Subscriber('autocharge_state_INO', String, self._station_subscriber_callback)
        imu_subscriber = rospy.Subscriber('imu', Imu, self._imu_subscriber_callback)
        autocharge_cancel_subscriber = rospy.Subscriber("charging_cancel", Bool, self._cancel_subscriber_callback)

        # srv
        self.turn_srv = rospy.ServiceProxy('/turn', TurnSrv)
        self.module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv) #hong

    def _autocharge_publisher(self, step_num):
        pub_rate = rospy.Rate(25) #5hz

        rospy.loginfo(step_num)
        self.autocharge_publisher.publish(step_num)
        pub_rate.sleep()

    def _velocity_publisher(self, flag, x, z):
        pub_rate = rospy.Rate(25) #5hz

        if flag:
            movemsg = Twist()
            movemsg.linear.x = x
            movemsg.angular.z = z

            self.velocity_publisher.publish(movemsg)
        else:
            pass

        pub_rate.sleep()

    def _battery_amount_subscriber_callback(self, msg):

        #print("SOC: ", msg.data.SOC)
        self.battery_amount = msg.data

    #def _battery2_amount_subscriber_callback(self, msg):
    #    self.battery2_amount = int(msg.data)

    def _station_subscriber_callback(self, msg):
        self.station_state = msg.data

        # start
        # contact
        # not_connected
        # left
        # right

    def _imu_subscriber_callback(self, msg):
        X, Y, Z = self._quaternion_to_euler_angle(msg.orientation)

        #degree range 0 ~ 359
        if Z > 0: 
            self.degree = int(Z) + 180
        else: 
            self.degree = int(Z) + 179

        '''
        print ("Quaternoion ================")
        print ("x : ", msg.orientation.x)
        print ("y : ", msg.orientation.y)
        print ("z : ", msg.orientation.z)
        print ("w : ", msg.orientation.w)
        print ("Euler -----------------------")
        print ("X : ", X)
        print ("Y : ", Y)
        print ("Z : ", Z)
        print("degree : ", self.degree)
        '''

    def _quaternion_to_euler_angle(self, msg):
        # ## for robotheading angle
        x = msg.x
        y = msg.y
        z = msg.z
        w = msg.w

        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return X, Y, Z

    def _cancel_subscriber_callback(self, data):
        self.cancel_flag = data.data    

    def movebase_client(self, x, y):

        print("movebase_client start!")
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        print("goal setup!")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1

        print("send goal position!")
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return client.get_result()


class AutochargeFunction:
    Ros_Func = RosFunction()
    recog = recongnition()

    direction_flag = False
    velocity_flag = False
    control_linear_vel = 0
    control_angular_vel = 0
    sequence = ""
    stop_time = 0

    def __init__(self):
        self.stop_flag = False
        self.sequence = "waiting"

    def init(self):
        self.center_check = '-'
        self.robot_position = "-"
        self.degree = 0
        self.distance = 0
        self.target_distance = 0

    def autocharge_main(self):
        if len(sys.argv) != 1:
            print("- Usage : python {}".format(sys.argv[0]))
            print("* Exiting...")

        while True:
            if self.Ros_Func.cancel_flag:
                return False

            if self.stop_flag:
                print("stop!!!!!!!!!!!!")
                self._stop_delay()

            if self.sequence == "waiting":
                self._waiting_sequence()
            elif self.sequence == "start":
                self._start_sequence()
            elif self.sequence == "search":
                self._search_sequence()
            elif self.sequence == "adjustment":
                self._adjustment_sequence()
            elif self.sequence == "guidance":
                self._guidance_sequence()
            elif self.sequence == "charging":
                print("SOC: ", self.Ros_Func.battery_amount)
                self._charging_sequence()
            elif self.sequence == "not_connected":
                print("SOC: ", self.Ros_Func.battery_amount)
                self._not_connected_sequence()
            elif self.sequence == "finish":
                print("SOC: ", self.Ros_Func.battery_amount)
                self._finish_sequence()
            else:
                self._else_sequence()

            if cv2.waitKey(1) == ord('q'):
                break

        self.finish()

    def _waiting_sequence(self):
        self.Ros_Func._autocharge_publisher(0)

##################hong
        f_led_mode = Led.Mode.on
        b_led_mode = Led.Mode.on
        f_led_color = Led.Color.red
        b_led_color = Led.Color.red

        command = "ledtest" + "_" + str(f_led_mode) + "_" + str(b_led_mode) + "_" +  str(f_led_color) + "_" +  str(b_led_color)

        self.Ros_Func.module_controller_srv(command)
        self.Ros_Func.module_controller_srv("uvc_off,air_off")
##################hong





        sleep(0.5)
        
        self.Ros_Func.movebase_client(0.41, 0.767)
        self.Ros_Func.turn_srv(230)
        self.sequence = "start"


    def _start_sequence(self):
        self.Ros_Func._autocharge_publisher(1)
        if self.Ros_Func.station_state == "start":
            self.sequence = "search"

    def _search_sequence(self):
        self.Ros_Func._autocharge_publisher(2)

        self.recog.image_processing()
        print("search_image_processing@@@@@@@@@@@@")
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.degree = int(self.recog.degree)
        self.distance = int(self.recog.distance)
        print("center_check: ", self.center_check)
        print("robot_position: ", self.robot_position)
        print("degree: ", self.degree)
        print("distance: ", self.distance)

        if self.center_check == '-':
            if self.direction_flag == False:
                self._left_turn(0.06, 0)
            elif self.direction_flag == True:
                self._right_turn(0.06, 0)
        elif self.center_check == 'RIGHT':
            self._right_turn(0.02, 0)
        elif self.center_check == 'LEFT':
            self._left_turn(0.02, 0)
        elif self.center_check == 'CENTER':
            self.stop_flag = True
            self.sequence = "adjustment"
            if self.robot_position == 'CENTER':
                self.stop_flag = True
                self.sequence = "guidance"
        else: pass

    def _adjustment_sequence(self):
        self.Ros_Func._autocharge_publisher(3)

        self.recog.image_processing()
        print("adjustment_image_processing@@@@@@@@@@@@")
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.degree = int(self.recog.degree)
        self.distance = int(self.recog.distance)
        self.target_distance = int(self.recog.target_distance)
        print("center_check: ", self.center_check)
        print("robot_position: ", self.robot_position)
        print("degree: ", self.degree)
        print("distance: ", self.distance)
        print("target_distance: ", self.target_distance)

        if self.center_check == '-':
            self.stop_flag = True
            self.sequence = "search"

        else:
            if self.robot_position == 'LEFT':
                self._left_turn(0.5, self.degree)
                self.direction_flag = True
                self.stop_flag = True
                self.sequence = "search"
                sleep(1)

            elif self.robot_position == 'RIGHT':
                self._right_turn(0.5, self.degree)
                self.direction_flag = False
                self.stop_flag = True
                self.sequence = "search"
                sleep(1)

            elif self.robot_position == 'CENTER':
                self.sequence = "guidance"

    def _guidance_sequence(self):
        self.Ros_Func._autocharge_publisher(4)

        self.recog.image_processing()
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.distance = int(self.recog.distance)
        print("center_check: ", self.center_check)
        print("robot_position: ", self.robot_position)
        print("distance: ", self.distance)
        
        if self.distance > 30:
            if self.robot_position == "CENTER":
                if self.center_check == "LEFT":
                    self._backward(0.02, 0.01)
                elif self.center_check == "RIGHT":
                    self._backward(0.02, -0.01)
                elif self.center_check == "CENTER":
                    self._backward(0.02, 0)

            elif self.robot_position == "LEFT":
                if self.center_check == "LEFT":
                    self._backward(0.02, -0.02)
                elif self.center_check == "RIGHT":
                    self._backward(0.02, 0.02)
                elif self.center_check == "CENTER":
                    self._backward(0.02, 0.01)

            elif self.robot_position == "RIGHT":
                if self.center_check == "LEFT":
                    self._backward(0.02, 0.02)
                elif self.center_check == "RIGHT":
                    self._backward(0.02, -0.02)
                elif self.center_check == "CENTER":
                    self._backward(0.02, -0.01)
            else:
                self.stop_flag = True

        else:
            if self.Ros_Func.station_state == "contact":

                ##################hong
                f_led_mode = Led.Mode.fade
                b_led_mode = Led.Mode.fade
                f_led_color = Led.Color.red
                b_led_color = Led.Color.red

                command = "ledtest" + "_" + str(f_led_mode) + "_" + str(b_led_mode) + "_" +  str(f_led_color) + "_" +  str(b_led_color)

                self.Ros_Func.module_controller_srv(command)
                ##################hong

                self.stop_flag = True
                self.sequence = "charging"
                self.recog.finish()
            else:
                self._backward(0.02, 0)
            """
            elif self.Ros_Func.station_state == "left":
                self._backward(0.005, -0.01)
            elif self.Ros_Func.station_state == "right":
                self._backward(0.005, 0.01)
            """

    def _charging_sequence(self):
        sleep(3)
        self.Ros_Func._autocharge_publisher(5)

        if self.Ros_Func.station_state == "not_connected":
            self.sequence = "not_connected"

        elif self.Ros_Func.station_state == "contact":
            if self.Ros_Func.battery_amount > 95:
                self.sequence = "finish"
        else:
            pass

    def _not_connected_sequence(self):
        self.Ros_Func._autocharge_publisher(6)
        print("not_connected!!!")
        self.finish()

    def _finish_sequence(self):
        self.Ros_Func._autocharge_publisher(7)
        print("finish!!!")

        sleep(3000)

        if self.Ros_Func.battery_amount > 95:
            self.sequence = "finish"

        else:
            self.sequence = "charging"
            pass

    def _else_sequence(self):
        self.Ros_Func._autocharge_publisher(8)
        print("else!!!")
        self.finish()

    def _stop_delay(self):
        if self.stop_time < 1:
            print("stop_time: ", self.stop_time)
            self._stop_turn()
            self.stop_time += 1
            sleep(1)

        else:
            self.stop_flag = False
            self.stop_time = 0

    def _stop_turn(self):
        self.Ros_Func._velocity_publisher(True, 0, 0)

    def _forward(self, velocity):
        self.Ros_Func._velocity_publisher(True, velocity, 0)
        print(int(self.recog.target_distance))
        sleep(int(self.recog.target_distance) * 0.4)

    def _backward(self, velocity, angular):
        self.Ros_Func._velocity_publisher(True, -velocity, angular)

    def _left_turn(self, velocity, _degree):
        if _degree != 0:
            _target_degree = self.Ros_Func.degree + 90

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.Ros_Func._velocity_publisher(True, 0, velocity)

                _degree = self.Ros_Func.degree

                print("degree: ", _degree)
                print("target_degree: ", _target_degree)

                if (_target_degree - 3) <= _degree <= (_target_degree + 3) :
                    self._stop_turn()
                    break

            print("first turn comple")

            sleep(0.3)
            self._forward(0.03)
            self._stop_turn()
            sleep(0.3)

            _target_degree = self.Ros_Func.degree - 65

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.Ros_Func._velocity_publisher(True, 0, -velocity)

                _degree = self.Ros_Func.degree

                print("degree: ", _degree)
                print("target_degree: ", _target_degree)

                if (_target_degree - 3) <= _degree <= (_target_degree + 3) :
                    self._stop_turn()
                    break

            self.recog.image_processing()
            sleep(1)
            print("second turn comple")
            
        else:
            self.Ros_Func._velocity_publisher(True, 0, velocity)

    def _right_turn(self, velocity, _degree):
        if _degree != 0:
            _target_degree = self.Ros_Func.degree - 90

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360


            while True:
                self.Ros_Func._velocity_publisher(True, 0, -velocity)

                _degree = self.Ros_Func.degree

                print("degree: ", _degree)
                print("target_degree: ", _target_degree)

                if (_target_degree - 3) <= self.Ros_Func.degree <= (_target_degree + 3) :
                    self._stop_turn()
                    break

            print("first turn comple")
            sleep(0.3)
            self._forward(0.03)
            self._stop_turn()
            sleep(0.3)

            _target_degree = self.Ros_Func.degree + 65

            if _target_degree <= 0:
                _target_degree = _target_degree + 360
            if _target_degree > 360:
                _target_degree = _target_degree - 360

            while True:
                self.Ros_Func._velocity_publisher(True, 0, velocity)

                _degree = self.Ros_Func.degree

                print("degree: ", _degree)
                print("target_degree: ", _target_degree)

                if (_target_degree - 3) <= self.Ros_Func.degree <= (_target_degree + 3) :
                    self._stop_turn()
                    break

            self.recog.image_processing()
            sleep(1)
            print("second turn comple")

        else:
            self.Ros_Func._velocity_publisher(True, 0, -velocity)

    def finish(self):
        self._stop_turn()
        self.recog.finish()
        sys.exit("exit")

if __name__ == '__main__':
    char_Func = AutochargeFunction()

    rospy.init_node('charging_server_example')

    char_Func.autocharge_main()
    rospy.spin()


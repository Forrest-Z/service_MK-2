#! /usr/bin/env python

import os
import sys 
import csv
import time
from time import sleep
import datetime
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
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu

#ros simple goal lib
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseResult
from zetabot_main.msg import MoveMsgs, MoveBaseActAction
from zetabot_main.srv import TurnSrv
from zetabot_main.msg import SonarArray
from zetabot_main.msg import ChargingAction,ChargingActionGoal,ChargingFeedback,ChargingActionResult
#from zetabot_main.msg import BatteryInformationMsgs

from zetabot_main.srv import ModuleControllerSrv # hong
from zetabot_main.srv import InitPoseSrv


from geometry_msgs.msg import PoseWithCovarianceStamped
from zetabot_main.srv import InitPoseSrv

initpose_srv = None
station_pose = rospy.get_param("/charging_station_pose")
init_pose = rospy.get_param("/robot_init_pose")


log_directory = "/home/zetabank/robot_log/autocharge_log"
today = time.strftime('%Y_%m_%d', time.localtime(time.time()))
file_name = log_directory + "/autocharge_log_"+today+".csv"

def initial_pos_pub():
    global init_pose

    print("#####################################################################################################")

    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = init_pose['position_x']
    start_pos.pose.pose.position.y = init_pose['position_y']
    start_pos.pose.pose.position.z = 1.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = init_pose['orientation_z']
    start_pos.pose.pose.orientation.w = init_pose['orientation_w']

    start_pos.pose.covariance[0] = 0.25
    start_pos.pose.covariance[7] = 0.25
    start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[35] = 0.06853891945200942

    print("#####################################################################################################")

    print(start_pos)

    rospy.loginfo(start_pos)
    rospy.sleep(1)
    publisher.publish(start_pos)
    rospy.sleep(1)

    return True




if os.path.isfile(file_name):
    print('ok')
else :
    if not os.path.exists(log_directory):
        os.makedirs(log_directory)
    f = open(file_name,'w')
    wr = csv.writer(f)
    log_name = ['Time', 'sequence']
    wr.writerow(log_name)
    f.close()

class RosFunction:
    cancel_flag = False
    battery_amount = 0.0
    sona_distance = 0
    degree = 0
    station_state = ""
    now_angle = 0

    def __init__(self):
        # publisher setup - move_vel
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.autocharge_publisher = rospy.Publisher('autocharge_state_NUC', UInt8, queue_size = 1)
        self.led_control_publisher = rospy.Publisher('/robot_mode', String, queue_size = 1)
        # subscriber setup
        battery_amount_subscriber = rospy.Subscriber('/battery_SOC', Float32, self._battery_amount_subscriber_callback)
        station_subscriber = rospy.Subscriber('autocharge_state_INO', String, self._station_subscriber_callback)
        imu_subscriber = rospy.Subscriber('imu', Imu, self._imu_subscriber_callback)
        autocharge_cancel_subscriber = rospy.Subscriber("charging_cancel", Bool, self._cancel_subscriber_callback)
        goal_result_subscriber = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self._movebase_result_subscriber_callback)
        sona_subscriber = rospy.Subscriber('/sonar', SonarArray, self._sona_subscriber_callback)
        #angle_subscriber = rospy.Subscriber('/angle', Int16, self._angle_subscriber_callback)

        # srv
        self.turn_srv = rospy.ServiceProxy('/turn', TurnSrv)
        self.initpose_srv = rospy.ServiceProxy("/init_pose_srv",InitPoseSrv)

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

    def _led_control_publisher(self, mode):
        pub_rate = rospy.Rate(25)

        rospy.loginfo(mode)
        self.led_control_publisher.publish(mode)
        pub_rate.sleep()

    def _battery_amount_subscriber_callback(self, msg):
        #print("SOC: ", msg.data.SOC)
        self.battery_amount = msg.data

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

    def _movebase_result_subscriber_callback(self, data):
        self.goal_status = data.status.status

    def _sona_subscriber_callback(self, data):
        self.sona_distance = int((data.data[5]+data.data[6])/2)
        #print(type(self.sona_distance))
        #print(self.sona_distance)

        """
        self.sona_left = data[5]
        self.sona_right = data[6]
        print("sona_left: ", self.sona_left)
        print("sona_right: ", self.sona_right)
        """

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

        self.goal_status = None
        print("wait_before: ", self.goal_status)
        
        print("send goal position!")
        client.send_goal(goal)
        print("send goal position!1111111")
        wait = client.wait_for_result()
        print("send goal position!22222222")

        print("wait_after: ", self.goal_status)
        print("send goal position!3333333")

        if self.goal_status == 3: pass
        else:
            self.movebase_client(station_pose["position_x"], station_pose["position_y"])

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

    search_fail_cnt = 0
    detect_fail_cnt = 0
    docking_fail_cnt = 0

    def __init__(self):
        self.stop_flag = False
        self.sequence = "waiting"
        self.finish_time_checker = False

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

        print("autocharge start!!!")

        while True:
            
            if self.Ros_Func.cancel_flag:
                self.Ros_Func.cancel_flag = False
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
        self.Ros_Func._autocharge_publisher(1)
        sleep(0.5)
        #####hong
        self.Ros_Func.movebase_client(station_pose["position_x"], station_pose["position_y"])
        self.Ros_Func.turn_srv(station_pose["degree"])
        #####hong
        self.detect_fail_cnt = 0
        self.search_fail_cnt = 0
        self.docking_fail_cnt = 0
        self.sequence = "start"

    def _start_sequence(self):
        self.Ros_Func._autocharge_publisher(1)
        if self.Ros_Func.station_state == "start":
            self.sequence = "search"
            f = open(file_name,'a')
            wr = csv.writer(f)

            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
            log = [now_time, self.sequence]

            wr.writerow(log)
            f.close()

    def _search_sequence(self):
        self.Ros_Func._autocharge_publisher(2)

        self.recog.image_processing()
        #print("search_image_processing@@@@@@@@@@@@")
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.degree = int(self.recog.degree)
        self.distance = int(self.recog.distance)
        #print("center_check: ", self.center_check)
        #print("robot_position: ", self.robot_position)
        #print("degree: ", self.degree)
        #print("distance: ", self.distance)

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
            f = open(file_name,'a')
            wr = csv.writer(f)

            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
            log = [now_time, "marker_detect_success"]
            wr.writerow(log)
            log = [now_time, self.sequence]
            wr.writerow(log)
            f.close()

            if self.robot_position == 'CENTER':
                self.stop_flag = True
                self.detect_fail_cnt = 0
                self.docking_fail_cnt = 0
                self.search_fail_cnt = 0
                self._stop_turn()
                self.sequence = "guidance"
                initial_pos_pub()
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, "success_search"]
                wr.writerow(log)
                log = [now_time, self.sequence]
                wr.writerow(log)
                f.close()
            else: pass
        else: pass

        if self.search_fail_cnt < 2000:
            print("search_fail_cnt: ", self.search_fail_cnt)
            self.search_fail_cnt += 1
        else:
            print("search_cancel")
            #self.Ros_Func.turn_srv(self.station_pose["reset_degree"])
            self._cancel_forward(0.05)
            self.direction_flag = False
            self.sequence = "waiting"

    def _adjustment_sequence(self):
        self.Ros_Func._autocharge_publisher(3)

        self.recog.image_processing()
        #print("adjustment_image_processing@@@@@@@@@@@@")
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.degree = int(self.recog.degree)
        self.distance = int(self.recog.distance)
        self.target_distance = int(self.recog.target_distance)
        #print("center_check: ", self.center_check)
        #print("robot_position: ", self.robot_position)
        #print("degree: ", self.degree)
        #print("distance: ", self.distance)
        #print("target_distance: ", self.target_distance)

        if self.center_check == '-':
            self.stop_flag = True
            self.sequence = "search"

        else:
            if self.robot_position == 'LEFT':
                self._left_turn(0.5, self.degree)
                self.direction_flag = True
                self.stop_flag = True
                self.sequence = "search"
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, "not_center_left"]
                wr.writerow(log)
                log = [now_time, self.sequence]
                wr.writerow(log)
                f.close()
                sleep(1)

            elif self.robot_position == 'RIGHT':
                self._right_turn(0.5, self.degree)
                self.direction_flag = False
                self.stop_flag = True
                self.sequence = "search"
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, "not_center_right"]
                wr.writerow(log)
                log = [now_time, self.sequence]
                wr.writerow(log)
                f.close()
                sleep(1)

            elif self.robot_position == 'CENTER':
                self.sequence = "guidance"
                initial_pos_pub()
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, self.sequence]

                wr.writerow(log)
                f.close()


                #self.Ros_Func.initpose_srv(init_pose["position_x"], init_pose["position_y"], init_pose["orientation_z"], init_pose["orientation_w"])




    def _guidance_sequence(self):
        self.Ros_Func._autocharge_publisher(4)

        self.recog.image_processing()
        self.center_check = self.recog.center_check
        self.robot_position = self.recog.robot_position
        self.distance = int(self.recog.distance)
        #print("center_check: ", self.center_check)
        #print("robot_position: ", self.robot_position)
        print("distance: ", self.distance)
        
        if self.Ros_Func.sona_distance < 8:
            #self.Ros_Func.turn_srv(self.station_pose["reset_degree"])
            self._cancel_forward(0.05)
            self.direction_flag = False
            self.sequence = "waiting"
            f = open(file_name,'a')
            wr = csv.writer(f)

            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
            log = [now_time, "sona_detect"]
            wr.writerow(log)
            log = [now_time, self.sequence]
            wr.writerow(log)
            f.close()

        elif self.Ros_Func.sona_distance > 40:
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
                if self.detect_fail_cnt < 10:
                    print("detect_fail_cnt: ", self.detect_fail_cnt)
                    self.detect_fail_cnt += 1
                else:
                    print("guidance_cancel")
                    #self.Ros_Func.turn_srv(self.station_pose["reset_degree"])
                    self._cancel_forward(0.05)
                    self.direction_flag = False
                    self.sequence = "waiting"
                    f = open(file_name,'a')
                    wr = csv.writer(f)

                    now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                    log = [now_time, "guidance_cancel"]
                    wr.writerow(log)
                    log = [now_time, self.sequence]
                    wr.writerow(log)
                    f.close()

        else:
            if self.Ros_Func.station_state == "contact":
                self.stop_flag = True
                self.sequence = "charging"
                self.Ros_Func._led_control_publisher("low_battery")
                self.recog.finish()
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, self.sequence]

                wr.writerow(log)
                f.close()
            elif self.Ros_Func.station_state == "left":
                self._backward(0.005, -0.01)
            elif self.Ros_Func.station_state == "right":
                self._backward(0.005, 0.01)
            else:
                self._backward(0.02, 0)

                if self.docking_fail_cnt < 200:
                    print("docking_fail_cnt: ", self.docking_fail_cnt)
                    self.docking_fail_cnt += 1
                else:
                    print("guidance_cancel")
                    #self.Ros_Func.turn_srv(self.station_pose["reset_degree"])
                    self._cancel_forward(0.05)
                    self.direction_flag = False
                    self.sequence = "waiting"
                    f = open(file_name,'a')
                    wr = csv.writer(f)

                    now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                    log = [now_time, "guidance_cancel"]
                    wr.writerow(log)
                    log = [now_time, self.sequence]
                    wr.writerow(log)
                    f.close()

    def _charging_sequence(self):
        sleep(3)
        self.Ros_Func._autocharge_publisher(5)

        #if self.Ros_Func.station_state == "disconnected":
        #    self.sequence = "disconnected"

        if self.finish_time_checker == False:
            self.finish_time = datetime.datetime.now() + datetime.timedelta(minutes= 25)
            self.finish_time_checker = True
        else:
            current_time = datetime.datetime.now()
            print("current_time: ", current_time)
            print("finish_time: ", self.finish_time)
            if current_time.minute == self.finish_time.minute:
                self.sequence = "finish"
                self.finish_time_checker = False


        if self.Ros_Func.station_state == "contact":

            if self.Ros_Func.battery_amount > 95:
                self.sequence = "finish"
                f = open(file_name,'a')
                wr = csv.writer(f)

                now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
                log = [now_time, self.sequence]

                wr.writerow(log)
                f.close()
        else:
            pass

    def _finish_sequence(self):
        self.Ros_Func._autocharge_publisher(6)
        print("finish!!!")
        sleep(1)

        if self.Ros_Func.battery_amount > 95:
            self.sequence = "finish"
            f = open(file_name,'a')
            wr = csv.writer(f)

            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
            log = [now_time, self.sequence]

            wr.writerow(log)
            f.close()
        """
        else:
            self.sequence = "charging"
            f = open(file_name,'a')
            wr = csv.writer(f)

            now_time = str(time.localtime(time.time()).tm_hour) + ":" + str(time.localtime(time.time()).tm_min)
            log = [now_time, self.sequence]

            wr.writerow(log)
            f.close()
            pass
        """
    def _disconnected_sequence(self):
        self.Ros_Func._autocharge_publisher(7)
        print("disconnected!!!")
        self.finish()

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

    def _cancel_forward(self, velocity):
        self.Ros_Func._velocity_publisher(True, velocity, 0)
        sleep(5)

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
        #sys.exit("exit")


class chargingAction(object):
    print("chargingAction start!!!")
    char_Func = AutochargeFunction()
    Ros_Func = RosFunction()
    # create messages that are used to publish feedback/result
    # _feedback = my_first_ros_pkg.msg.TesttFeedback()
    _result = ChargingActionResult()

    def __init__(self, name):
        self.clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ChargingAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # append the seeds for the fibonacci sequence
        
        # publish info to the console for the user
        # rospy.loginfo('%s: Executing, creating charging sequence of order %s ' % (self._action_name, goal.order))
        idx = 0
        # self._feedback.feedback = my_first_ros_pkg.msg.TesttActionFeedback()

        #write to charging flow
        char_Func = AutochargeFunction()
        #os.system("mplayer ~/voice/charging_start.mp3")
        print("autocharge_main start!!!")
        success = char_Func.autocharge_main()
        
        # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
        self.clear_costmaps_srv()
        
        if success:
            #os.system("mplayer ~/voice/charging_done.mp3")
            self._result.result = "end_autocharge_mode"
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        else:
            #os.system("mplayer ~/voice/charging_cancel.mp3")
            self._result.result = "cancel_charging"
            self.Ros_Func._led_control_publisher("air_condition")
            rospy.loginfo('%s: cancled' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    char_Func = AutochargeFunction()

    rospy.init_node('charging_server_example')
    server = chargingAction("charging_act")
    print ("00000000000000000000000000000000")
    rospy.spin()


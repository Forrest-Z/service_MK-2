#! /usr/bin/env python
import rospy

import os
import json

import actionlib

import sys

sys.path.append("/home/zetabank/catkin_ws/devel/lib/python2.7/dist-packages")
sys.path.append("/opt/ros/melodic/lib/python2.7/dist-packages")
sys.path.append("/home/zetabank/.local/lib/python2.7/site-packages")

from apscheduler.jobstores.base import JobLookupError
from apscheduler.schedulers.background import BackgroundScheduler
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from zetabot_main.msg import MoveMsgs
from zetabot_main.msg import ScheduleAirAction, ScheduleAirGoal
from zetabot_main.msg import ScheduleFullcoverageAction, ScheduleFullcoverageGoal
from zetabot_main.msg import ChargingAction,ChargingActionGoal,ChargingFeedback,ChargingActionResult, ChargingGoal
# from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from actionlib_msgs.msg import GoalStatusArray
from zetabot_main.srv import ModuleControllerSrv, TurnQuaternionSrv
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Quaternion
import threading

#import select, termios, tty

full_path = []


cur_mode = 'rest'
air_result = ''
floor_result = ''
charging_result =''

move_base_result_status = None
move_base_fail_cnt = 0



cancle_result = ['cancel_air_condition_mode','cancel_fulcoverage','cancel_charging']

air_cancel_pub = rospy.Publisher('air_condition_cancel', Bool, queue_size=10)
floor_cancel_pub = rospy.Publisher('floor_cleaning_cancel', Bool, queue_size=10)
charging_cancel_pub = rospy.Publisher('charging_cancel', Bool, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
robot_mode_pub = rospy.Publisher('/robot_mode',String,queue_size=10)
power_ctl_pub =  rospy.Publisher('power_ctl', String, queue_size=10)

module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)
turn_quaternion_srv = rospy.ServiceProxy('/turn/quaternion', TurnQuaternionSrv)


def initial_pos_pub():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('initial_pos_pub', anonymous=True)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = -0.846684932709
    start_pos.pose.pose.position.y = 0.333061099052
    start_pos.pose.pose.position.z = 1.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = -1.123
    start_pos.pose.pose.orientation.w = 0.719166613815

    start_pos.pose.covariance[0] = 0.25
    start_pos.pose.covariance[7] = 0.25
    start_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    start_pos.pose.covariance[35] = 0.06853891945200942

    rospy.loginfo(start_pos)
    rospy.sleep(1)
    publisher.publish(start_pos)
    rospy.sleep(1)

def cancel_mod_pub(mode):
    global air_result
    global floor_result
    global charging_result
    global cancle_result
    global cur_mode
    wait_result = ''
    _cancel_result =''
    
    if mode =='air_cleaning':
        air_cancel_pub.publish(True)
        print ("test2222")
        _cancel_result = cancle_result[0]
    elif mode =='floor_cleaning':
        floor_cancel_pub.publish(True)
        _cancel_result = cancle_result[1]
    elif mode == 'charging' :
        charging_cancel_pub.publish(True)
        _cancel_result = cancle_result[2]

    cnt = 0
    
    while True :
        print ("test111")
        if mode == 'air_cleaning':
            wait_result = air_result

        elif mode == 'floor_cleaning':
            wait_result = floor_result
        elif mode == 'charging':
            wait_result = charging_result
            print "test2......... %s" % wait_result
            print "test33333 %s" % _cancel_result
        if cnt >= 10 or wait_result == _cancel_result:
            air_result =''
            floor_result=''
            charging_result=''
            break
        rospy.sleep(1)
        cnt+=1

def batterty_callback(data):
    global cur_mode
    global charging_result
    #battery full > 1100
    if int(data.data) < 400 and cur_mode != 'charging':
        charging_client()
    if int(data.data) > 1100 and cur_mode == 'charging' :
        charging_cancel_pub.publish(True)

def charging_client():
    global cur_mode
    global charging_result
    if cur_mode == 'air_condition':
        cur_mode = 'charging'
        cancel_mod_pub('air_cleaning')
    elif cur_mode == 'full_coverage':
        cur_mode = 'charging'
        cancel_mod_pub('floor_cleaning')
    elif cur_mode == 'rest':
        cur_mode = 'charging'


    # Creates the SimpleActionClient, passing the type of the action
    # (chargingAction) to the constructor.
    client = actionlib.SimpleActionClient('charging_act', ChargingAction)
    # Waits until the action server has started up and started
    # listening for goals.
    print ('wait for charging_server')
    client.wait_for_server()
    print ('wait for charging_server111')
    # Creates a goal to send to the action server.
    goal = ChargingGoal()

    # Sends the goal to the action server.
    robot_mode_pub.publish("charging")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print ('wait for charging_server22')
    client.wait_for_result()
   
    # Prints out the result of executing the action
    #charging end operat
    charging_result = client.get_result().result
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.03
    cmd_vel.angular.z = 0.00

    cmd_vel_pub.publish(cmd_vel)

    rospy.sleep(5)

    cmd_vel.linear.x = 0.00
    cmd_vel.angular.z = 0.00

    cmd_vel_pub.publish(cmd_vel)

    cur_mode = 'rest'
    return  charging_result # A chargingResult

def air_cleaning_client():
    global cur_mode
    global air_result
    if cur_mode == 'rest' :
        cur_mode ='air_condition'
        print(cur_mode)
        # Creates the SimpleActionClient, passing the type of the action
        # (chargingAction) to the constructor.
        client = actionlib.SimpleActionClient('/air_condition_action', ScheduleAirAction)

        
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        # Creates a goal to send to the action server.
        goal = ScheduleAirGoal()
        goal.start_flag = True

        # Sends the goal to the action server.
        
        robot_mode_pub.publish("air_condition")
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        
        air_result =client.get_result().result
        cur_mode = 'rest'
        # Prints out the result of executing the action
        print 'test air action return %s' % air_result

        return air_result # A chargingResult


def floor_cleaning_client():
    global cur_mode
    global floor_result
    if  cur_mode== 'rest' or cur_mode =='air_condition':
        cur_mode = 'full_coverage'
        cancel_mod_pub('air_cleaning')


        print("full_coverage")
        # Creates the SimpleActionClient, passing the type of the action
        # (chargingAction) to the constructor.
        client = actionlib.SimpleActionClient('/fullCoverage_action', ScheduleFullcoverageAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = ScheduleFullcoverageGoal()
        robot_mode_pub.publish("full_coverage")
        # Sends the goal to the action server.
        client.send_goal(goal)

        # Waits for the server to finish performing the action.
        client.wait_for_result()

        # Prints out the result of executing the action
        floor_result = client.get_result().result  # A chargingResult

        cur_mode ='rest'
        print("rest")
        return floor_result

def movebase_client(x,y,z=1):
    global move_base_fail_cnt


    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

    move_base_fail_cnt = 0

    rospy.sleep(1)

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = z

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

class Scheduler(object):
    def __init__(self):
        self.sched = BackgroundScheduler()
        self.sched.start()
        self.job_id=''
        self.make_time(4,8)
        self.index = 0


    def __del__(self):
       self.shutdown()

    #all_kill jobs
    def shutdown(self):
       self.sched.shutdown()

    #kill_job
    def kill_scheduler(self, job_id):
        try:
            self.sched.remove_job(job_id)
        except JobLookupError as err:
            print "fail to stop scheduler: %s" % err
        return

    #operate func
    def hello(self, type,job_id):
        global  cur_mode
        if cur_mode == 'rest' :
            air_cleaning_client()


    def air_clean(self,type, job_id):
        global  cur_mode
        if cur_mode == 'rest' :
            air_cleaning_client()

    def floor_clean(self,type, job_id): #full coverage
        print("floor `~~~~")
        result = floor_cleaning_client()
        print("Result:" + result)

    def battery_charge(self,type, job_id):
        global cur_mode
        cur_mode = "charging"
        result = charging_client()
        print("Result:" + result.result)
        if result.result == 'charging_cancel':
            print("charging_cancel result" + result.result)
        cur_mode = 'rest'

    def charging_cancel(self,type, job_id):
        global  cur_mode
        if cur_mode == 'charging':
            cancel_mod_pub('charging')
            cur_mode = 'rest'
            print('charging cancel')
        else:
            print('Not charging')       

    def make_time(self,charging_term,charging_cancel_term) :
        hour_now = time.localtime(time.time()).tm_hour
        min_now = time.localtime(time.time()).tm_min
        
        if (min_now + 1)//60 :
            self.floor_min = (min_now + 1)%60
            self.floor_hour = hour_now + ((min_now + 1)//60)
        else :
            self.floor_min = min_now + 1
            self.floor_hour = hour_now
        
        if (min_now + charging_term)//60 :
            self.charging_min = (min_now + charging_term)%60
            self.charging_hour = hour_now + ((min_now + charging_term)//60)
        else :
            self.charging_min = min_now + charging_term
            self.charging_hour = hour_now

        if (min_now + charging_cancel_term)//60 :
            self.charging_cancel_min = (min_now + charging_cancel_term)%60
            self.charging_cancel_hour = hour_now + ((min_now + charging_cancel_term)//60)
        else :
            self.charging_cancel_min = min_now + charging_cancel_term
            self.charging_cancel_hour = hour_now
        
    def roming_move(self,type, job_id) :
        global cur_mode
        if cur_mode == 'rest' or cur_mode == 'air_condition':
            print("call_back")
            # roming_list = [  
            #     {"x" : 6.0, "y" : 1.46 },
            #     {"x" : 6.42, "y" : -2.81 },
            #     {"x" : 2.8, "y" : 1.46 },
            #     {"x" : 3.15, "y" : -2.6 }
            #     ] ###gwang ju 
            
            roming_list = [
                {"x" : 0.42, "y" : -1.74 },
                {"x" : 4.35, "y" : 0.023 },
                {"x" : 3.316, "y" : 2.301 },
                {"x" : 0.66, "y" : -0.66 }
                ] #hub

            robot_mode_pub.publish("air_condition")           
            result = movebase_client(roming_list[self.index]["x"],roming_list[self.index]["y"])
            
            self.index += 1
            if self.index >= 4 :
                self.index = 0

    def fullcoverage(self,type, job_id) :
        global full_path
        global cur_mode
        global move_base_result_status
        global end_flag

        start_flag = True
        move_flag = True
        cnt = 0

        # roming_list = [  
        #     {"x" : 6.0, "y" : 1.46 },
        #     {"x" : 6.42, "y" : -2.81 },
        #     {"x" : 2.8, "y" : 1.46 },
        #     {"x" : 3.15, "y" : -2.6 }
        #     ] ###gwang ju 
            
        # roming_list = [
        #     {"x" : -5.568, "y" : 0.400, "z" : 84 },
        #     {"x" : -0.581, "y" : -3.209, "z" : 84 }
        #     ] #example



        rospy.sleep(1)
        
        while True :
            
            rospy.sleep(0.1)
            robot_mode_pub.publish("normal")
            if start_flag :

                if cur_mode == 'rest' :
                    cur_mode = 'full_coverage'
                    self.index = 0

                    start_flag = False

                    while cnt != 4 :
                        move_flag = False
                        result = movebase_client(full_path[self.index]["position"]["x"],full_path[self.index]["position"]["y"])

                        if int(move_base_result_status) == 3:
                            move_flag = True
                            break

                        else:
                            cnt += 1
                    cnt = 0

                    pre_x, pre_y, pre_w, pre_z = full_path[self.index]["position"]["x"], full_path[self.index]["position"]["y"],  full_path[self.index]["orientation"]["w"],  full_path[self.index]["orientation"]["z"], 

                    self.index += 1

                    if self.index >= len(full_path) :
                        self.index = 0
                    

                while cur_mode == 'full_coverage' :
                    print("call_back")


                    robot_mode_pub.publish("full_coverage")


                    print("x :",full_path[self.index]["position"]["x"])
                    print("y :",full_path[self.index]["position"]["y"])

                    if pre_x != full_path[self.index]["position"]["x"] or pre_y != full_path[self.index]["position"]["y"] :
                        while cnt != 4 :
                            move_flag = False
                            result = movebase_client(full_path[self.index]["position"]["x"],full_path[self.index]["position"]["y"])

                            if int(move_base_result_status) == 3:
                                move_flag = True
                                break

                            else:
                                cnt += 1
                        cnt = 0


                    elif (pre_w != full_path[self.index]["orientation"]["w"] or pre_z != full_path[self.index]["orientation"]["z"]) and move_flag:
                        orientation = Quaternion()
                        
                        orientation.x = full_path[self.index]["orientation"]["x"]
                        orientation.y = full_path[self.index]["orientation"]["y"]
                        orientation.z = full_path[self.index]["orientation"]["z"]
                        orientation.w = full_path[self.index]["orientation"]["w"]
                        
                        turn_quaternion_srv(orientation)



                    pre_x, pre_y, pre_w, pre_z = full_path[self.index]["position"]["x"], full_path[self.index]["position"]["y"],  full_path[self.index]["orientation"]["w"],  full_path[self.index]["orientation"]["z"], 


                    self.index += 1

                    cnt = 0
                    if self.index >= len(full_path) :
                        self.index = 0


                self.index = 0

            else :
                rospy.sleep(0.2)



    def go_home(self,type,job_id) :
        robot_mode_pub.publish("charging")
        result = movebase_client(-0.46,0.167)    



    def scheduler(self,type, job_id):

        if job_id == 'job_id':
            self.sched.add_job(self.hello, type, seconds=10, id=job_id, args=('interval',job_id))
        elif job_id == 'air_condition':
            # self.sched.add_job(self.air_clean, type, seconds=10, id=job_id, args=('interval',job_id))
            print("sche")
            self.sched.add_job(self.roming_move, type, seconds=30, id=job_id, args=('interval',job_id))
        elif job_id == 'roming_':
            self.sched.add_job(self.roming_move, type, seconds=30, id=job_id, args=('interval',job_id))
        elif job_id == 'floor_cleaning':
            self.sched.add_job(self.floor_clean, 'cron', day_of_week='mon-fri',hour=str(self.floor_hour),minute=str(self.floor_min) , id='floor_cleaning', args=('cron',job_id))
        elif job_id == 'charging' :
            #self.sched.add_job(self.battery_charge, type, seconds=10, id=job_id, args=('interval',job_id))
            self.sched.add_job(self.battery_charge, 'cron', day_of_week='mon-fri', hour='9,10,11,13,14,15,16,17,18', minute='0, 10, 20, 30, 40, 50', id='charging', args=('cron',job_id))
        elif job_id == 'charging_cancel' :
            self.sched.add_job(self.charging_cancel, 'cron', day_of_week='mon-fri', hour='9,10,11,13,14,15,16,17,18', minute='5, 15, 25, 35, 45, 55', id='charging_cancel', args=('cron',job_id))
        elif job_id == 'charging_noon' :
            #self.sched.add_job(self.battery_charge, type, seconds=10, id=job_id, args=('interval',job_id))
            self.sched.add_job(self.battery_charge, 'cron', day_of_week='mon-fri', hour='12', minute='0', id='charging_noon', args=('cron',job_id))
        elif job_id == 'charging_cancel_noon' :
            self.sched.add_job(self.charging_cancel, 'cron', day_of_week='mon-fri', hour='13', minute='10', id='charging_cancel_noon', args=('cron',job_id))  

def recv_move_base_result(msg) :
    global move_base_result_status

    move_base_result_status = msg.status.status

def recv_move_base_status(msg) :
    global move_base_fail_cnt

    if len(msg.status_list) >= 1 :
        if msg.status_list[0].status == 4 :
            move_base_fail_cnt += 1

    if move_base_fail_cnt == 10 :
        data = Bool

        data.data = True
        emergency_stop_pub.publish(data)

        clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
        clear_costmaps_srv()

        move_base_fail_cnt = 0
        rospy.sleep(1)

        data.data = False
        emergency_stop_pub.publish(data)

def full_path_load() :
    global full_path
    file_path = "/home/zetabank/catkin_ws/src/zetabot_main/path/path_V2.json"

    with open(file_path, 'r') as json_file:
        full_path = json.load(json_file)

    del full_path[0]
    del full_path[1]

if __name__ == '__main__':
    rospy.init_node('robot_schedule')
    rospy.Subscriber("battery",String, batterty_callback)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, recv_move_base_result)
    # rospy.Subscriber("/move_base/status",GoalStatusArray,recv_move_base_status)


    scheduler = Scheduler()

    # module_controller_srv("air_lv2_on")

    # scheduler.scheduler('cron', "floor_cleaning")

    # scheduler.scheduler('cron', "charging")
    # scheduler.scheduler('cron', "charging_cancel")

    # scheduler.scheduler('cron', "charging_noon")
    # scheduler.scheduler('cron', "charging_cancel_noon")
    # scheduler.scheduler('interval', "roming_")

    # scheduler.scheduler('cron', "charging_cancel")

    # scheduler.scheduler('interval', "air_condition")

    full_path_load()

    scheduler.fullcoverage('cron','full')

    # charging_client()

    # count = 0
    # while True:
    #     # print "Running main process..............."
    #     rospy.sleep(1)
    #     count += 1
    #     #scheduler.kill_scheduler("1")
    #     # print "######### kill cron schedule ##########"

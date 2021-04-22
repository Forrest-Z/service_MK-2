#! /usr/bin/env python
import rospy

import os

import actionlib
from apscheduler.jobstores.base import JobLookupError
from apscheduler.schedulers.background import BackgroundScheduler
import time
from std_msgs.msg import Bool
from std_msgs.msg import String
from zetabot_main.msg import MoveMsgs
from zetabot_main.msg import ScheduleAirAction, ScheduleAirGoal
from zetabot_main.msg import ScheduleFullcoverageAction, ScheduleFullcoverageGoal
# from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from zetabot_main.srv import ModuleControllerSrv, TurnSrv
from std_srvs.srv import Empty
import my_first_ros_pkg.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from socket import *


cur_mode = 'rest'
air_result = ''
floor_result = ''
charging_result =''

cancle_result = ['cancel_air_condition_mode','cancel_fulcoverage','cancel_charging']

air_cancel_pub = rospy.Publisher('air_condition_cancel', Bool, queue_size=10)
floor_cancel_pub = rospy.Publisher('floor_cleaning_cancel', Bool, queue_size=10)
charging_cancel_pub = rospy.Publisher('charging_cancel', Bool, queue_size=10)
move_vel_pub = rospy.Publisher('/move_vel',MoveMsgs,queue_size=10)
robot_mode_pub = rospy.Publisher('/robot_mode',String,queue_size=10)
power_ctl_pub =  rospy.Publisher('power_ctl', String, queue_size=10)

turn_srv = rospy.ServiceProxy('turn', TurnSrv)



module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)

serverSock = socket(AF_INET, SOCK_STREAM)
serverSock.bind(('192.168.112.2',8080))
# serverSock.bind(('192.168.43.60',8080))
serverSock.listen(1)
move_base_result_status = None

connectionSock, addr = serverSock.accept()

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

# def charging_client():
#     global cur_mode
#     global charging_result
#     if cur_mode == 'air_condition':
#         cur_mode = 'charging'
#         cancel_mod_pub('air_cleaning')
#     elif cur_mode == 'full_coverage':
#         cur_mode = 'charging'
#         cancel_mod_pub('floor_cleaning')
#     elif cur_mode == 'rest':
#         cur_mode = 'charging'


#     # Creates the SimpleActionClient, passing the type of the action
#     # (chargingAction) to the constructor.
#     client = actionlib.SimpleActionClient('charging_act', ChargingAction)
#     # Waits until the action server has started up and started
#     # listening for goals.
#     print ('wait for charging_server')
#     client.wait_for_server()
#     print ('wait for charging_server111')
#     # Creates a goal to send to the action server.
#     goal = ChargingGoal()

#     # Sends the goal to the action server.
#     robot_mode_pub.publish("charging")
#     client.send_goal(goal)

#     # Waits for the server to finish performing the action.
#     print ('wait for charging_server22')
#     client.wait_for_result()
   
#     # Prints out the result of executing the action
#     #charging end operat
#     charging_result = client.get_result().result
#     move_vel = MoveMsgs()
#     move_vel.header.frame_id = 'charging'
#     move_vel.linear_x = 0.03
#     move_vel.angular_z = 0.00

#     move_vel_pub.publish(move_vel)

#     rospy.sleep(5)

#     move_vel.header.frame_id = 'charging'
#     move_vel.linear_x = 0.00
#     move_vel.angular_z = 0.00

#     move_vel_pub.publish(move_vel)

#     cur_mode = 'rest'
#     return  charging_result # A chargingResult

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


    clear_costmaps_srv = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    clear_costmaps_srv()

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

def recv_move_base_result(msg) :
    global move_base_result_status

    move_base_result_status = msg.status.status

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
        global connectionSock
        global move_base_result_status

        cnt = 0

        # roming_list = [  
        #     {"x" : 6.0, "y" : 1.46 },
        #     {"x" : 6.42, "y" : -2.81 },
        #     {"x" : 2.8, "y" : 1.46 },
        #     {"x" : 3.15, "y" : -2.6 }
        #     ] ###gwang ju 
            
#         roming_list = [
#             {"x" : -5.604, "y" : -4.105, "z" : 84 },
#             {"x" : -5.694, "y" : -3.622, "z" : 84 },
#             {"x" : -4.198, "y" : -11.997, "z" : 338 },
#             {"x" : -3.216, "y" : -0.66, "z" : 0 },
#             {"x" : -5.278, "y" : -0.66, "z" : 0 }
#             ] #example

#         1 : x : -5.60440178216y : -4.10512694604z : 0.01
# 2 : x : -5.69443403474y : -2.00302750495z : 0.01
# 3 : x : -4.19825455821y : -2.18175119315z : 0.01
# 4 : x : -3.21655457727y : -3.51095214234z : 0.01
# 5 : x : -5.2789029404y : -5.93232939075z : 0.01

        roming_list = [
            {"x" : 31.827, "y" : -13.402, "z" : 86 },
            {"x" : 23.706, "y" : -8.914, "z" :  179},
            {"x" : 13.925, "y" : -11.946, "z" : 86 },
            {"x" : 2.968, "y" : -14.259, "z" :  305},
            {"x" : -0.428, "y" : -15.641, "z" : 264 }
            # {"x" : 0.66, "y" : -0.66, "z" : 0 }
            ] #hub


        if cur_mode == 'rest' :
            cur_mode = 'roming_move'


            input_msg = None


            while not input_msg == 1 :
                input_msg = input("next(1)")

            turn_srv(205)

            input_msg = None


            while not input_msg == 1 :
                input_msg = input("next(1)")

            msg = str(9)
            print("msg : ",msg)
            connectionSock.send(msg.encode('utf-8'))
            print("send_msg : ",msg)

            robot_mode_pub.publish("face_detect")

            rospy.sleep(3)


            robot_mode_pub.publish("air_condition")

            result = movebase_client(roming_list[1]["x"],roming_list[1]["y"])
            turn_srv(roming_list[1]["z"])

            msg = str(8)
            print("msg : ",msg)
            connectionSock.send(msg.encode('utf-8'))
            print("send_msg : ",msg)

            robot_mode_pub.publish("face_detect")
            recv_msg = ''

            input_msg = None

            while not input_msg == 1 :
                input_msg = input("next(1)")

            msg = str(5)
            print("msg : ",msg)
            connectionSock.send(msg.encode('utf-8'))
            print("send_msg : ",msg)

            input_msg = None

            while not input_msg == 1 :
                input_msg = input("next(1)")


            msg = str(7)
            print("msg : ",msg)
            connectionSock.send(msg.encode('utf-8'))
            print("send_msg : ",msg)

            recv_msg = ''
            while recv_msg != 'end' :
                recv_msg = connectionSock.recv(1024).decode("utf-8")
                print(recv_msg)

            robot_mode_pub.publish("air_condition")

        while cur_mode == 'roming_move' :
            print("call_back")


            robot_mode_pub.publish("air_condition")
            result = movebase_client(roming_list[self.index]["x"],roming_list[self.index]["y"])

            rospy.sleep(1)
            print("done")

            print("result : " + str(move_base_result_status))

            if int(move_base_result_status) == 3 :
                cnt = 0
                print("in_turn")
                turn_srv(roming_list[self.index]["z"])
                msg = str(self.index)
                print("msg : ",msg)
                connectionSock.send(msg.encode('utf-8'))
                print("send_msg : ",msg)

                robot_mode_pub.publish("face_detect")
                recv_msg = ''
                while recv_msg != 'end' :
                    recv_msg = connectionSock.recv(1024).decode("utf-8")
                    print(recv_msg)

            else :
                cnt += 1
                if cnt >= 3 :
                    None
                else :
                    continue
                
            

            self.index += 1
            if self.index >= len(roming_list) :
                self.index = 0


        self.index = 0

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
            # self.sched.add_job(self.roming_move, type, seconds=30, id=job_id, args=('interval',job_id))
            # self.sched.add_job(self.roming_move, 'cron', day_of_week='tue-sun', hour='9,13', minute='0', id=job_id, args=('cron',job_id))
            self.sched.add_job(self.roming_move, 'cron', day_of_week='mon-sun', hour='19', minute='32', id=job_id, args=('cron',job_id))
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



if __name__ == '__main__':
    rospy.init_node('robot_schedule')
    rospy.Subscriber("battery",String, batterty_callback)

    rospy.Subscriber("/move_base/result", MoveBaseActionResult, recv_move_base_result)

    scheduler = Scheduler()

    module_controller_srv("air_lv2_on")

    module_controller_srv("uvc_on")

    # scheduler.scheduler('cron', "floor_cleaning")

    # scheduler.scheduler('cron', "charging")
    # scheduler.scheduler('cron', "charging_cancel")

    # scheduler.scheduler('cron', "charging_noon")
    # scheduler.scheduler('cron', "charging_cancel_noon")
    # scheduler.scheduler('interval', "roming_")

    # scheduler.scheduler('cron', "charging_cancel")
    print("ready")

    scheduler.scheduler('cron', "roming_")

    count = 0
    # while True:
    #     # print "Running main process..............."
    #     rospy.sleep(1)
    #     count += 1
    #     #scheduler.kill_scheduler("1")
    #     # print "######### kill cron schedule ##########"

    rospy.spin()

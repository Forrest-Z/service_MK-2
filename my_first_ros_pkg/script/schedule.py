#! /usr/bin/env python
import rospy

import os,thread, threading
import subprocess, shlex
import actionlib
from apscheduler.jobstores.base import JobLookupError
from apscheduler.schedulers.background import BackgroundScheduler
import time
from std_msgs.msg import Bool
from std_msgs.msg import String,UInt16
from geometry_msgs.msg import Twist
from zetabot_main.msg import ScheduleAirAction, ScheduleAirGoal
from zetabot_main.msg import ScheduleFullcoverageAction, ScheduleFullcoverageGoal
from zetabot_main.srv import InitPoseSrv
from autocharge.msg import ChargingAction, ChargingActionGoal, ChargingGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from zetabot_main.srv import ModuleControllerSrv
from std_srvs.srv import Empty
import my_first_ros_pkg.msg
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import UInt8
import sys, select, termios, tty
import random


cur_mode = 'rest'
air_result = ''
floor_result = ''
charging_result =''
pre_charge_stat = ''

srv = None


cancle_result = ['cancel_air_condition_mode','cancel_fulcoverage','cancel_charging']

air_cancel_pub = rospy.Publisher('air_condition_cancel', Bool, queue_size=10)
floor_cancel_pub = rospy.Publisher('floor_cleaning_cancel', Bool, queue_size=10)
charging_cancel_pub = rospy.Publisher('charging_cancel', Bool, queue_size=10)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
robot_mode_pub = rospy.Publisher('/robot_mode',String,queue_size=10)
power_ctl_pub =  rospy.Publisher('power_ctl', String, queue_size=10)


module_controller_srv = rospy.ServiceProxy("/module_controller_srv", ModuleControllerSrv)

def os_system(path):
    os.system(path)

def charging_cancel_poweron():
    power_ctl_pub.publish("on")
    rospy.sleep(10)
    thread.start_new_thread(os_system,('roslaunch zetabank_bringup zetabank_robot.launch',))
    rospy.sleep(10)

def charge_state_callback(data):
    global pre_charge_stat
    if pre_charge_stat == 4 and data.data == 5 :
        pre_charge_stat = data.data    
        print "444444444444555555555555555"
        power_ctl_pub.publish("off")
        rospy.sleep(10)
        print os.system("pkill -INT -ef zetabank_bringup")
    # if pre_charge_stat == 5 and data.data == 6 or data.data == 7:
    #     pre_charge_stat = data.data
    #     print "555555555555555666666666666"
    #     #power_ctl_pub.publish("on")
    #     rospy.sleep(10)
    #     #thread.start_new_thread(os_system,('roslaunch zetabank_bringup zetabank_robot.launch',))
    pre_charge_stat = data.data
    

def initial_pos_pub():
    publisher = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")
    start_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    start_pos.header.frame_id = "map"
    start_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    start_pos.pose.pose.position.x = 2.174
    start_pos.pose.pose.position.y = 1.315
    start_pos.pose.pose.position.z = 1.0

    start_pos.pose.pose.orientation.x = 0.0
    start_pos.pose.pose.orientation.y = 0.0
    start_pos.pose.pose.orientation.z = 2.123
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
    global cur_mode
    global srv
    cur_mode = "charging"
    print("charging_mode_on")
    
    
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
    print ("ttttttttttttttttttttttttt")
    cmd_vel = Twist()
    cmd_vel.linear.x = 0.03
    cmd_vel.angular.z = 0.00

    cmd_vel_pub.publish(cmd_vel)

    rospy.sleep(5)
    print ("ttttttttttttttttttttttttt22222222222222")
    cmd_vel.linear.x = 0.00
    cmd_vel.angular.z = 0.00

    cmd_vel_pub.publish(cmd_vel)
    print ("ttttttttttttttttttttttttt333333333")
    cur_mode = 'rest'

    print("Result:" + charging_result)
    if charging_result == 'charging_cancel':
        print("charging_cancel result" + charging_result)
    cur_mode = 'rest'
    #srv(-1.0974,-0.855,-0.1844,0.982)


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
        print("charging_mode_on")
        result = charging_client()
        print("Result:" + result.result)
        if result.result == 'charging_cancel':
            print("charging_cancel result" + result.result)
        cur_mode = 'rest'

    def charging_cancel(self,type, job_id):
        global  cur_mode
        if cur_mode == 'charging':
            #charging_cancel_poweron()
            #rospy.sleep(20)
            #initial_pos_pub()
            cancel_mod_pub('charging')
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
            
            # roming_list = [
            #     {"x" : 0.42, "y" : -1.74 },
            #     {"x" : 4.35, "y" : 0.023 },
            #     {"x" : 3.316, "y" : 2.301 },
            #     {"x" : 0.66, "y" : -0.66 }
            #     ] #hub

            roming_list = [
                {"x" : 0.272, "y" : -1.7254315 }
                ] #gj_3f  2.174, 1.427 -1.423, "y" : -1.019 }

            robot_mode_pub.publish("air_condition")           
            result = movebase_client(roming_list[self.index]["x"],roming_list[self.index]["y"])
            
            self.index += 1
            if self.index >= len(roming_list) :
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
            self.sched.add_job(self.roming_move, type, seconds=20, id=job_id, args=('interval',job_id))
        elif job_id == 'roming_':
            self.sched.add_job(self.roming_move, type, seconds=30, id=job_id, args=('interval',job_id))
        elif job_id == 'floor_cleaning':
            self.sched.add_job(self.floor_clean, 'cron', day_of_week='mon-fri',hour=str(self.floor_hour),minute=str(self.floor_min) , id='floor_cleaning', args=('cron',job_id))
        elif job_id == 'charging' :
            #self.sched.add_job(self.battery_charge, type, seconds=10, id=job_id, args=('interval',job_id))
            self.sched.add_job(self.battery_charge, 'cron', day_of_week='mon-fri', hour='9,10,11,12,13,14,15,16,17,18', minute='0, 10, 20, 30, 40, 50', id='charging', args=('cron',job_id))
        elif job_id == 'charging_cancel' :
            self.sched.add_job(self.charging_cancel, 'cron', day_of_week='mon-fri', hour='9,10,11,12,13,14,15,16,17,18', minute='5, 15, 25, 35, 45, 55', id='charging_cancel', args=('cron',job_id))
        elif job_id == 'charging_noon' :
            #self.sched.add_job(self.battery_charge, type, seconds=10, id=job_id, args=('interval',job_id))
            self.sched.add_job(self.battery_charge, 'cron', day_of_week='mon-fri', hour='12', minute='0', id='charging_noon', args=('cron',job_id))
        elif job_id == 'charging_cancel_noon' :
            self.sched.add_job(self.charging_cancel, 'cron', day_of_week='mon-fri', hour='13', minute='10', id='charging_cancel_noon', args=('cron',job_id))  

def voice_random() :
    path = "/home/zetabank/voice_gj/"
    print(os.listdir(path))

    pre_num = -1

    voice_list = os.listdir(path)
    while True :
        if cur_mode != 'charging' :
            cur_num = random.randrange(0,5)
            if cur_num != pre_num :
                voice = "mplayer " + path +voice_list[cur_num]
                result = os.system(voice)
                pre_num = cur_num
                time.sleep(5)
            else :
                time.sleep(0.1)
        else :
            time.sleep(1)

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('robot_schedule')
    rospy.Subscriber("battery",UInt16, batterty_callback)
    srv = rospy.ServiceProxy("/init_pose_srv",InitPoseSrv)
    #rospy.Subscriber("autocharge_state_NUC", UInt8, charge_state_callback)
    scheduler = Scheduler()

   

    # scheduler.scheduler('cron', "floor_cleaning")
    # scheduler.scheduler('cron', "charging_noon")
    # scheduler.scheduler('cron', "charging_cancel_noon")
    # scheduler.scheduler('interval', "roming_")

    # scheduler.scheduler('cron', "charging_cancel")

    t1 = threading.Thread(target=voice_random)
    t1.daemon = True 
    t1.start()
    
    
    #module_controller_srv("air_lv2_on")
    scheduler.scheduler('interval', "air_condition")
    scheduler.scheduler('cron', "charging")
    scheduler.scheduler('cron', "charging_cancel")

   
    

    count = 0


    while(1):
        key = getKey()
        if key == 'a' :
            scheduler.roming_move("tt","tt")
        elif key == 's' :
            t1 = threading.Thread(target=charging_client)
            t1.daemon = True 
            t1.start()
        elif key == 'd' :
            global  cur_mode
            print ("dddddddddddddd")
            if cur_mode == 'charging':
                #charging_cancel_poweron()
                #rospy.sleep(20)
                #initial_pos_pub()
                cancel_mod_pub('charging')
                cur_mode = 'rest'
                print('charging cancel')
            else:
                print('Not charging')     
            

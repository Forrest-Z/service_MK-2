#!/usr/bin/env python
import rospy

from std_msgs.msg import String

from full_coverage.srv import Fullpath
from zetabot_main.srv import ModuleControllerSrv
from threading import Thread

#-------------------------------------------------
# def module_controller(*args):

#     for i in args :
#         module_controller_srv(i)
#         print("module_controll",i)
#         rospy.sleep(0.1)

# module_controller("UVC_on","LED_off","LEDG_on")
#-------------------------------------------------

module_controll_command = 0b0000000000000000
module_controll_command_str = "0000"

module_controller_topic = "/module_control_NUC"
module_control_pub = rospy.Publisher(module_controller_topic,String,queue_size=10)

def module_controller(comm):
    global module_controll_command
    global module_controll_command_str

    command_str = comm.command
    module_controll_command_str = String()


    # |     RESERVE   |       LED        |    Pulifier      |  pump     |Reserve| UVC |
    # | 0  1  2  3  4 |  5     6    7    |  8   9   10  11  | 12   13   |  14   | 15  |
    # | X  X  X  X  X |  LEDG  LEDB LEDR |  L4  L3  L2  L1  | SOL  PUMP |  X    | UVC |


    # LED R, G, B   9, 5, 3
    

    module_controll_dict = {
        "pump" : 0b0000000000001100,
        "UVC"  : 0b0000000000000001,
        "LEDG" : 0b0000010100000000,
        "LEDB" : 0b0000001100000000,
        "LEDR" : 0b0000100100000000,
        "LED"  : 0b0000111100000000,
        "all"  : 0b1111111111111111
    }

    

    # for i in args :
    #     if "_off" in i :
    #         print(i[:-1*(len("_off"))])
    #         command = ~ module_controll_dict[i[:-1*(len("_off"))]]
    #         module_controll_command = command & module_controll_command
    #     elif "_on" in i :
    #         print(i[:-1*(len("_on"))])
    #         command = module_controll_dict[i[:-1*(len("_on"))]]
    #         module_controll_command = command | module_controll_command

    if "_off" in command_str :
        command = ~ module_controll_dict[command_str[:-1*(len("_off"))]]
        module_controll_command = command & module_controll_command
    elif "_on" in command_str :
        command = module_controll_dict[command_str[:-1*(len("_on"))]]
        module_controll_command = command | module_controll_command
        

    module_controll_command_str = str(hex(module_controll_command)).upper()[2:].zfill(4)
    
    


    print(command_str)

    return (command_str)

def pub_thread() :
    while True :
        print(module_controll_command_str)
        module_control_pub.publish(module_controll_command_str)
        rospy.sleep(0.5)

def main() :
    rospy.init_node("module_controller_server")

    rospy.sleep(1)

    pubThrd = Thread(target=pub_thread, args=())  # thread to process received packets
    pubThrd.daemon = True
    pubThrd.start()

    srv = rospy.Service('module_controller_srv', ModuleControllerSrv, module_controller)
    print("Module_control Ready")

    rospy.spin()

if __name__ == "__main__":
    main()

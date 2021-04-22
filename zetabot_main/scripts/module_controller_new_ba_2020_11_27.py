#!/usr/bin/env python
import rospy

from std_msgs.msg import String

from full_coverage.srv import Fullpath
from zetabot_main.srv import ModuleControllerSrv
from zetabot_main.msg import ModuleControlmsgs
from threading import Thread

#-------------------------------------------------
# from zetabot_main.srv import ModuleControllerSrv

# module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)


# module_controller_srv("uvc_on,led_off,led_green,air_lv2,air_off")
#-------------------------------------------------

module_controller_topic = "/module_control_NUC"
module_control_pub = rospy.Publisher(module_controller_topic,ModuleControlmsgs,queue_size=10)

module_controll_command = ModuleControlmsgs()

def module_controller(comm_list):
    global module_controll_command

    comm_list = comm_list.command.split(",")




    #| pump | sol | uvc | sonar_UF | sonar_UR | sonar_UB | sonar_UL | sonar_DF | sonar_DR | sonar_DB | sonar_DL |
    
    module_control_target_dict = {
        "pump" : 0,
        "sol" : 1,
        "uvc" : 2,
        "sonar" : {
            "UF" : 3,
            "UR" : 4,
            "UB" : 5,
            "UL" : 6,
            "DF" : 7,
            "DR" : 8,
            "DB" : 9,
            "DL" : 10
        }
    }

    led_color_RGB = {
        "red" : [255,0,0],
        "green" : [0,255,0],
        "blue" : [0,0,255],
        "orange" : [255,128,0],
        "off" : [0,0,0]
    }

    pulifier_level = {
        "off" : 0,
        "lv1" : 100,
        "lv2" : 300,
        "lv3" : 500
    }

    for i in comm_list :
        comm = i.split("_")
        print(i)
        if "led" in i :
            print("led")
            module_controll_command.led = led_color_RGB[comm[1]]

        elif "air" in i :
            print("air")
            module_controll_command.pulifier = pulifier_level[comm[1]]

        elif "all" == comm[0] : 
            if "_on" in i :
                module_controll_command.led = led_color_RGB["green"]
                module_controll_command.pulifier = pulifier_level["lv3"]
                for k in range(0,11) : 
                    module_controll_command.module_power[k] = True
            elif "_off" in i :
                module_controll_command = ModuleControlmsgs()
        else :
            if comm[0] == "sonar" : 
                print("sonar")
                if comm[1] == "all" : 
                    if "_on" in i :
                        for key,value in module_control_target_dict[comm[0]].items() :
                            module_controll_command.module_power[value] = True
                    elif "_off" in i :
                        for key,value in module_control_target_dict[comm[0]].items() :
                            module_controll_command.module_power[value] = False
                else : 
                    print("else")
                    if "_on" in i :
                        module_controll_command.module_power[module_control_target_dict[comm[0]][comm[1]]] = True
                    elif "_off" in i :
                        module_controll_command.module_power[module_control_target_dict[comm[0]][comm[1]]] = False

            if "on" in i :
                module_controll_command.module_power[module_control_target_dict[comm[0]]] = True
            elif "off" in i :
                module_controll_command.module_power[module_control_target_dict[comm[0]]] = False
                
    module_control_pub.publish(module_controll_command)

    print(module_controll_command)

    # print(command_str)

    return (str(module_controll_command))


def pub_thread() :
    None
    # while True :
    #     print(module_controll_command_str)
    #     module_control_pub.publish(module_controll_command_str)
    #     rospy.sleep(0.5)

def main() :
    rospy.init_node("module_controller_server")

    rospy.sleep(1)

    pubThrd = Thread(target=pub_thread, args=())  # thread to process received packets
    pubThrd.daemon = True
    pubThrd.start()

    module_controll_command = ModuleControlmsgs()

    module_control_pub.publish(module_controll_command)

    print(module_controll_command)

    srv = rospy.Service('module_controller_srv', ModuleControllerSrv, module_controller)
    print("Module_control Ready")
    print("-"*20)

    rospy.spin()

if __name__ == "__main__":
    main()

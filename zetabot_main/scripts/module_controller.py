#!/usr/bin/env python
import rospy
import time

from std_msgs.msg import String, UInt16

from full_coverage.srv import Fullpath
from zetabot_main.srv import ModuleControllerSrv
from zetabot_main.msg import ModuleControlMsgs
from threading import Thread

#-------------------------------------------------
# from zetabot_main.srv import ModuleControllerSrv

# module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)


# module_controller_srv("uvc_on,led_off,led_green,air_lv2,air_off")
#-------------------------------------------------

module_controller_topic = "/module_control_NUC"
purifier_topic = "/purifier_command"
module_control_pub = rospy.Publisher(module_controller_topic,ModuleControlMsgs,queue_size=10)
purifier_pub = rospy.Publisher(purifier_topic,UInt16,queue_size=10)

module_controll_command = ModuleControlMsgs()
purifier_command = UInt16()

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

led_count_number = 69

blink_flag = False
led_color = [0,0,0]
blink_term = 0
led_count = 0

led_mode = {
    "off" : 0,
    "spin" : 255
}

led_color_RGB = {
    "red" : [255,0,0],
    "green" : [0,255,0],
    "blue" : [0,0,255],
    "orange" : [255,128,0]
}



pulifier_level = {
    "off" : 0,
    "lv1" : 100,
    "lv2" : 250,
    "lv3" : 400
}

def module_controller(comm_list):
    global module_controll_command
    global blink_flag
    global blink_term
    global led_color
    global led_count

    comm_list = comm_list.command.split(",")



    #| pump | sol | uvc | sonar_UF | sonar_UR | sonar_UB | sonar_UL | sonar_DF | sonar_DR | sonar_DB | sonar_DL |
    
    #led_off
    #led_red_30
    #led_blue_spin
    #led_blink_on_1
    #led_blink_off

    for i in comm_list :
        comm = i.split("_")
        print(i)
        if "led" in i :
            print("led")
            if 'off' == comm[1] :
                blink_flag = False
                module_controll_command.led = [0,0,0,0]
                continue
            elif 'spin' in comm : 
                led_count = 69
                blink_flag = False
                led_command = led_color_RGB[comm[1]][:]
                led_command.append(led_mode[comm[2]])
            elif comm[1] == "blink" : 
                if comm[2] == "off" :
                    blink_flag = False
                    module_controll_command.led[3] = led_count
                    continue
                elif comm[2] == "on" :
                    blink_flag, blink_term = True, comm[3]
                    continue
            else :
                led_count = int(comm[2]) * led_count_number / 100
                led_command = led_color_RGB[comm[1]][:]
                if led_count > 255:
                    led_count = led_count_number
                led_command.append(led_count)
            module_controll_command.led = led_command
            

        elif "air" in i :
            print("air")
            purifier_command.data = pulifier_level[comm[1]]

        elif "all" == comm[0] : 
            if "_on" in i :
                module_controll_command.led = led_color_RGB["green"]
                module_controll_command.pulifier = pulifier_level["lv3"]
                for k in range(0,11) : 
                    module_controll_command.module_power[k] = True
            elif "_off" in i :
                module_controll_command = ModuleControlMsgs()
        
        elif "uvc" == comm[0] :
            if "_on" in i :
                module_controll_command.module_power[module_control_target_dict[comm[0]]] =True
            elif "_off" in i :
                module_controll_command.module_power[module_control_target_dict[comm[0]]] = False
            print("")

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
                

    print(module_controll_command)

    # print(command_str)

    return (str(module_controll_command))


def led_blink():
    None
    # global module_controll_command
    # global blink_flag
    # global blink_term
    # global led_color
    # i = True
    # start_time = 0
    # k=0

    # while True :
    #     if not blink_flag :
    #         continue

    #     elif start_time == 0 :
    #         print("on")
    #         start_time = time.time()
    #         led_blink_count = int(i) * led_count
    #         module_controll_command.led[3] = int(led_blink_count)
    #         module_controll_command.time_stamp = k
    #         print(module_controll_command)
    #         k += 1

    #     elif (time.time() - start_time) >= float(blink_term) :
    #         i = not(i)
    #         led_blink_count = int(i) * led_count
    #         module_controll_command.led[3] = int(led_blink_count)
    #         start_time = time.time()
    #         module_controll_command.time_stamp = k
    #         print(module_controll_command)
    #         k += 1
        

        


def pub_thread() :
    global module_controll_command
    global purifier_pub
    # while True :
    #     print(module_controll_command_str)
    #     module_control_pub.publish(module_controll_command_str)
    #     rospy.sleep(0.5)
    while True :
        try:
            module_control_pub.publish(module_controll_command)
            purifier_pub.publish(purifier_command)
        except :
            print("eeeeee",module_controll_command)
        rospy.sleep(0.1)
    

def main() :
    rospy.init_node("module_controller_server")

    rospy.sleep(1)

    pubThrd = Thread(target=pub_thread, args=())  # thread to process received packets
    pubThrd.daemon = True
    pubThrd.start()

    led_blink_Thrd = Thread(target=led_blink, args=())  # thread to process received packets
    led_blink_Thrd.daemon = True
    led_blink_Thrd.start()

    module_controll_command = ModuleControlMsgs()

    module_control_pub.publish(module_controll_command)

    print(module_controll_command)

    srv = rospy.Service('module_controller_srv', ModuleControllerSrv, module_controller)
    print("Module_control Ready")
    print("-"*20)

    rospy.spin()

if __name__ == "__main__":
    main()

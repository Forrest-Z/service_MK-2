#!/usr/bin/env python
import rospy

from zetabot_main.srv import ModuleControllerSrv


module_controller_srv = rospy.ServiceProxy("/module_controller_srv",ModuleControllerSrv)


def module_controller(*args):
    for i in args :
        module_controller_srv(i)
        print("module_controll",i)
        rospy.sleep(0.1)



def main():

    rospy.init_node("test")
    while True :
        """
        module_controller_srv("pump_on,air_lv2")
        rospy.sleep(1)
        module_controller_srv("pump_off")
        rospy.sleep(1)
        module_controller_srv("ledr_on")
        rospy.sleep(0.5)
        module_controller_srv("ledo_on")
        rospy.sleep(0.5)
        module_controller_srv("ledb_on")
        rospy.sleep(0.5)
        """
        # module_controller_srv("pump_off")
        rospy.sleep(0.1)
        module_controller_srv("pump_off")
        rospy.sleep(0.5)
        # module_controller_srv("led_red")
        # rospy.sleep(0.1)
        # module_controller_srv("led_orange")
        # rospy.sleep(0.1)
        # module_controller_srv("led_blue")
        # rospy.sleep(0.1)
        # # module_controller_srv("pump_on")
        # rospy.sleep(0.1)
        module_controller_srv("pump_on")
        rospy.sleep(0.5)
    rospy.spin()

if __name__ == "__main__":
    main()

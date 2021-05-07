#!bin/bash

source /opt/ros/melodic/setup.bash
source /home/zetabank/catkin_ws/devel/setup.bash

export ROS_MASTER_URI=http://192.168.112.2:11311
export ROS_HOSTNAME=192.168.112.2


sleep(1)
rosrun rosserial_python serial_node.py _baud:=460800 _port:=/dev/$(ls -lrt /dev | grep ^l | grep stlinkv2-1 |grep ttyACM | awk '{print$(NF-2)}') __name:='stm'

sleep(1)
rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/$(ls -lrt /dev | grep ^l | grep PWR |grep ttyUSB | awk '{print$(NF-2)}') __name:='PWR'

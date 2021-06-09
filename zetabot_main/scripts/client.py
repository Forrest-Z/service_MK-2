#! /usr/bin/env python

import rospy
# requests import -> pip install requests
import requests

rospy.init_node("server_client")

rospy.get_param("server_url","")

url = rospy.get_param("server_url","")
files = {'file': open('/home/zetabank/robot_log/air_log/air_log_2021_06_08.csv', 'rb')}
params = {"file":"sensor_0608_new.csv", "description":"test_1"}
getdata = requests.post(url, params, files=files)
print(getdata.text)

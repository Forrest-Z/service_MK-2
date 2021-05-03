status_data = {
            "battery" : None,
            "emergency" : None,
            "robot_mode" : None,
            "charging" : None,
            "speak" : None
        }

for i in status_data :
    if i.values() == None :
        print(status_data.keys())


print(status_data)
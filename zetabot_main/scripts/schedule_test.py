import time
import threading


def test_func() :
    print(time.localtime(time.time()).tm_hour,time.localtime(time.time()).tm_min,time.localtime(time.time()).tm_sec)

def test_func2() :
    print(time.localtime(time.time()).tm_hour,time.localtime(time.time()).tm_min,time.localtime(time.time()).tm_sec)



class Schedule :
    def __init__(self) :
        None

    def add_schdule(self,func,schdule_mode,days_of_week,start_time,term=0,end_time=0) :
        '''
        schdule_mode = interval, cron, once

        term = sec  if you use interval mode.

        start_time = "15:30"
                     ["10:30","12:40","15:00"]     cron mode
        end_time = "18:00:
        

        '''

        if schdule_mode == "interval" :
            new_thread = threading.Thread(target=self.interval_thread,args=(func,start_time,end_time,term),daemon=True)
            new_thread.start()
            None
        elif schdule_mode == "cron" :
            new_thread = threading.Thread(target=self.cron_thread,args=(func,start_time),daemon=True)
            new_thread.start()    
            None
        elif schdule_mode == "once" :
            func()
            None

    def cron_thread(self,func,start_time) :
        start_time_list = []

        for i in start_time :
            start_time_list.append(i.split(":"))

        while True :
            for i in start_time :
                

    def interval_thread(self,func,start_time,end_time,term) :

        print("make")
        pre_time = 0
        start_time = start_time.split(":")
        end_time = end_time.split(":")

        while True :
            if time.localtime(time.time()).tm_hour == start_time[0] and time.localtime(time.time()).tm_min == start_time[1] :
                while time.localtime(time.time()).tm_hour == end_time[0] and time.localtime(time.time()).tm_min == end_time[1] :
                    if time.localtime(time.time()).tm_sec - pre_time >= term :
                        test_func()
                        pre_time = time.localtime(time.time()).tm_sec
                        time.sleep(0.3)

    def schedule_info(self) :
        None

    def remove_schdule(self) :
        None


sch = Schedule()
sch.add_schdule(test_func,"interval","09:40","09:50",2)
try :
    while True :
        time.sleep(1)
except KeyboardInterrupt :
    print("exit")
import re
import os

class CameraUsbSetup():
    def detect_index(self):
        device_index = None

        for video_idx_file in os.listdir("/sys/class/video4linux"):
            video_file_path = os.path.realpath("/sys/class/video4linux/" + video_idx_file)

            for video_files in os.listdir(video_file_path):
                if 'name' in video_files:
                    name = open(os.path.realpath(video_file_path + '/' + 'name'), 'r').readline()

                    if re.match('KINGSEN', name) != None:
                        device_index = int(video_idx_file[-1])

        return device_index


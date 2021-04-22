#!/usr/bin/env python

"""
    Direction API

    20200220 kyuhsim - created
"""

# Python 2/3 compatibility
from __future__ import print_function

import sys, getopt
from time import sleep
from math import sin, pi

import cv2

from video import create_capture
from common import clock, draw_str
from calc_direction import image_analysis, image_analysis_centermark, get_marklist, image_center_centermark

# ## configurations that can be changed
gImgMaxWidth = 640
gImgMaxHeight = 480

# Direction API class
class Direction:
    def __init__(self):
        #self.cam = None
        #self.fallback = 'synth:bg=../data/lena.jpg:noise=0.05'
        self.fallback = ''
        self.numimg_i = 0
        self.b = 0

        self.inited = False
        self.debug_show = False

        self.light_red1 = (170, 70, 50)        # red wraps around 180
        self.dark_red1  = (180, 255, 255)
        self.light_red2 = (0, 70, 50)
        self.dark_red2  = (10, 255, 255)
        self.light_blue = (95, 50, 45)    #(110, 70, 50)  #(110, 50, 50)
        self.dark_blue  = (140, 255, 255)    #(130, 255, 255)
        self.light_yellow = (20, 100, 100)
        self.dark_yellow  = (45, 255, 255)    #(40, 255, 255)

        self.args, self.video_src = getopt.getopt(sys.argv[1:], '', ['mindist=', 'param1=', 'param2=', 'minradius=', 'maxradius='])
        #try:
        #    self.video_src = self.video_src[0]
        #except:
        #    self.video_src = 0
        self.video_src = 0
        self.args = dict(self.args)
        print("- args:", self.args)

        self.mindist = self.args.get('--mindist', 30)
        self.param1 = self.args.get('--param1', 100)
        self.param2 = self.args.get('--param2', 18)
        self.minradius = self.args.get('--minradius', 10)
        self.maxradius = self.args.get('--maxradius', 50)   # 30
        print("- mindist:", self.mindist, ", param1:", self.param1, ", param2:", self.param2, ", minradius:", self.minradius, ", maxradius:", self.maxradius)

        self.cam = create_capture(self.video_src, self.fallback)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def init(self):
        #self.cam = create_capture(self.video_src, self.fallback)
        self.inited = True
        print("- inited", self.inited)

    def check_direction(self):
        result = 'fail'
        ret, img = self.cam.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv_img, self.light_yellow, self.dark_yellow)
        result_img = cv2.bitwise_and(img, img, mask=mask_yellow)
        gray_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)
        vis_img = result_img.copy()
        result, x, y, r, n = image_analysis_centermark(gray_img, self.mindist, self.param1, self.param2, self.minradius, self.maxradius)

        if 'fail' == result:
            print("- result is fail")
            if self.debug_show:
                draw_str(img, (20, 20), 'no circle found')
                if (img is not None) and (img.size > 0):
                    cv2.imshow('yellow_vis', img)
                if 0xFF & cv2.waitKey(5) == 27:  # without waitkey(), imshow() does not work
                    pass
                sleep(0.05)
            '''print("- return fail")'''
            return result

        if self.debug_show:
            cv2.circle(vis_img, (x, y), r, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(vis_img, (x, y), 2, (0, 255, 0), 3, cv2.LINE_AA)
            draw_str(vis_img, (20, 20), '(%d, %d, %d), num:%d' % (x, y, r, n))
            if (vis_img is not None) and (vis_img.size > 0):
                cv2.imshow('yellow_vis', vis_img)
            if 0xFF & cv2.waitKey(5) == 27:    # without waitkey(), imshow() does not work
                pass
            sleep(0.05)
        return result

    def check_center(self, margin_=5):
        result = 'fail'
        ret, img = self.cam.read()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv_img, self.light_yellow, self.dark_yellow)
        result_img = cv2.bitwise_and(img, img, mask=mask_yellow)
        gray_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)
        vis_img = result_img.copy()
        result, x, y, r, n = image_center_centermark(gray_img, self.mindist, self.param1, self.param2, self.minradius, self.maxradius, margin_)

        if 'fail' == result:
            print("- result : fail")
            if self.debug_show:
                draw_str(img, (20, 20), 'no circle found')
                if (img is not None) and (img.size > 0):
                    cv2.imshow('yellow_vis', img)
                if 0xFF & cv2.waitKey(5) == 27: 
                    pass
                sleep(0.02)
            return result

        if self.debug_show:
            cv2.circle(vis_img, (x, y), r, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(vis_img, (x, y), 2, (0, 255, 0), 3, cv2.LINE_AA)
            draw_str(vis_img, (20, 20), '(%d, %d, %d), num:%d' % (x, y, r, n))
            if (vis_img is not None) and (vis_img.size > 0):
                cv2.imshow('yellow_vis', vis_img)
            if 0xFF & cv2.waitKey(5) == 27:
                pass
            sleep(0.02)
        return result

    def get_direction(self):
        move_direction_str = '-'
        move_amount = 0    # cm
        distance = 35    # cm
        prop = 1.0
        deg = 0.0

        ret, img = self.cam.read()
        t = clock()
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask_yellow = cv2.inRange(hsv_img, self.light_yellow, self.dark_yellow)

        center_result_img = cv2.bitwise_and(img, img, mask=mask_yellow)
        center_gray_img = cv2.cvtColor(center_result_img, cv2.COLOR_BGR2GRAY)
        center_gray_img = cv2.medianBlur(center_gray_img, 5)
        center_result, x, y, r, n = image_analysis_centermark(center_gray_img, self.mindist, self.param1, self.param2, self.minradius, self.maxradius)

        if 'fail' == center_result:
            return 'fail', 'center', '0', '0', '0'

        distance = self.r_to_distance(int(r))
        '''print("- distance:",  distance, ", r:", r)'''

        mask_blue = cv2.inRange(hsv_img, self.light_blue, self.dark_blue)
        mask_red1 = cv2.inRange(hsv_img, self.light_red1, self.dark_red1)
        mask_red2 = cv2.inRange(hsv_img, self.light_red2, self.dark_red2)
        mask_red  = mask_red1 | mask_red2

        final_mask = mask_yellow + mask_blue + mask_red

        #result_white_img = cv2.bitwise_and(img, img, mask=mask_white)
        result_img = cv2.bitwise_and(img, img, mask=final_mask)

        gray_img = cv2.cvtColor(result_img, cv2.COLOR_BGR2GRAY)
        gray_img = cv2.medianBlur(gray_img, 5)
        self.numimg_i += 1
        vis_img = result_img.copy()
        circles = image_analysis(gray_img, self.mindist, self.param1, self.param2, self.minradius, self.maxradius)
        try:
            a, b, c = circles.shape
        except:
            print("no circle ", self.numimg_i)
            return 'fail', 'center', '0', '0', '0'

        for i in range(b):
            cv2.circle(vis_img, (circles[0][i][0], circles[0][i][1]), circles[0][i][2], (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(vis_img, (circles[0][i][0], circles[0][i][1]), 2, (0, 255, 0), 3, cv2.LINE_AA)

        mark_list, center_index = get_marklist(circles, x, y, r, b)
        '''print("- got mark_list", mark_list, ", len:", len(mark_list), ", center_index:", center_index)'''

        if len(mark_list)-1 > center_index and center_index > 0:
            left_distance = mark_list[center_index][0] - mark_list[center_index-1][0]
            right_distance = mark_list[center_index+1][0] - mark_list[center_index][0]
            if abs(right_distance - left_distance) < 5:    # hard-wired
                move_direction_str = 'center'
                if right_distance > left_distance:
                    prop = right_distance / left_distance
                elif right_distance < left_distance:
                    prop = left_distance / right_distance
                '''print("- center: prop:", prop)'''
            elif right_distance > left_distance:
                move_direction_str = 'left'
                #move_amount = right_distance - left_distance
                prop = right_distance / left_distance
                deg = self.prop_to_deg(prop)
                move_amount = self.deg_to_amount(deg, distance)
                '''print("- prop:", prop, ", deg:", deg, ", amount:", move_amount)'''
            elif right_distance < left_distance:
                move_direction_str = 'right'
                #move_amount = left_distance - right_distance
                prop = left_distance / right_distance
                deg = self.prop_to_deg(prop)
                move_amount = self.deg_to_amount(deg, distance)
                '''print("- prop.:", prop, ", deg:", deg, ", amount:", move_amount)'''
        else:
            print("* cannot get neighbors.")

        dt = clock() - t
        draw_str(vis_img, (20, 20), 'time: %.1f ms, num: %d, %dth' % (dt*1000, b, self.numimg_i))

        if self.debug_show:
            cv2.imshow('vis', vis_img)
            #cv2.imshow('gray_img', gray_img)    # debug        if self.debug_show:
            #cv2.imshow('orig', img)
            if 0xFF & cv2.waitKey(5) == 27:    # without waitkey(), imshow() does not work
                pass
            sleep(0.1)
        return 'success', move_direction_str, str(move_amount), str(distance), str(deg)

    def set_circlesize(self, size=50):
        self.maxradius = size

    def get_circlesize(self):
        return self.maxradius

    def debug_show_img(self, show=True):
        self.debug_show = show

    def r_to_distance(self, r_):
        if r_ > 49:
            return 15
        elif r_ in range(47,50):
            return 17
        elif r_ in range(39,47):
            return 19
        elif r_ in range(34,39):
            return 23
        elif r_ in range(31,34):
            return 25
        elif r_ in range(27,31):
            return 27
        elif r_ in range(24,27):
            return 30
        elif r_ in range(22,24):
            return 33
        elif r_ in range(20,22):
            return 35
        elif r_ in range(19,20):
            return 38
        elif r_ in range(17,19):
            return 40
        elif r_ < 17:
            return 45

    def prop_to_deg(self, prop_):
        if prop_ == 1.0:
            return 0
        elif prop_ > 1.0 and prop_ <= 1.026:
            return 0
        elif prop_ > 1.026 and prop_ <= 1.040:
            return 1
        elif prop_ > 1.040 and prop_ <= 1.067:
            return 2
        elif prop_ > 1.067 and prop_ <= 1.098:
            return 3
        elif prop_ > 1.098 and prop_ <= 1.113:
            return 4
        elif prop_ > 1.113 and prop_ <= 1.135:
            return 5
        elif prop_ > 1.135 and prop_ <= 1.165:
            return 6
        elif prop_ > 1.165 and prop_ <= 1.200:
            """ """
            return 7
        elif prop_ > 1.200 and prop_ <= 1.210:
            return 8
        elif prop_ > 1.210 and prop_ <= 1.233:
            return 9
        elif prop_ > 1.233 and prop_ <= 1.265:
            """ """
            return 10
        elif prop_ > 1.265 and prop_ <= 1.290:
            return 11
        elif prop_ > 1.290 and prop_ <= 1.383:
            return 14
        elif prop_ > 1.383 and prop_ <= 1.453:
            return 15
        elif prop_ > 1.453 and prop_ <= 1.500:
            return 16
        elif prop_ > 1.500 and prop_ <= 1.571:
            return 17
        elif prop_ > 1.571 and prop_ <= 1.876:
            return 18
        elif prop_ > 1.876 and prop_ <= 2.200:
            return 19
        elif prop_ > 2.200:
            return 20

    def deg_to_amount(self, deg_, dist_):
        amount = dist_ * sin(deg_ * pi / 180)
        '''print("- deg:", deg_, ", dist:", dist_, ", amount:", amount)'''
        return amount

    def fini(self):
        self.inited = False
        cv2.destroyAllWindows()
        print("- finied", self.inited)


if __name__ == '__main__':
    import sys, getopt

    api = Direction()
    api.init()

    sleep(0.1)
    api.check_direction()    # discard to settle up camera

    api.debug_show_img(True)
    for i in range(50):
        api.check_direction()
        print("- after check_direction()")
        api.check_center(5)
        print("- after check_center()")
        api.get_direction()
        print("- after get_direction()")
    api.fini()

# #### EOF #####

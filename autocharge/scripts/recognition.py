# Python 2/3 compatibility
from __future__ import print_function

import cv2
import sys
import numpy as np
import math
from time import time, sleep
import cgitb

cgitb.enable(format = 'text')

class recongnition:
    capture = cv2.VideoCapture(0)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    fps = capture.get(cv2.CAP_PROP_FPS) 

    try:
        delay = int(1000/fps)
    except ZeroDivisionError:
        print("ZeroDivisionError")
        delay = 40

    degree = 0.0
    distance = 0.0
    target_distance = 0.0
    center_check = "-"
    robot_position = "-"
        
    MIN_AREA, MAX_AREA = 50, 20000
    MIN_WIDTH, MAX_WIDTH = 5, 150
    MIN_HEIGHT, MAX_HEIGHT = 10, 100
    MIN_POINT, MAX_POINT = 100, 400
    MIN_RATIO, MAX_RATIO = 0.5, 1.3
    MAX_AREA_DIFF = 0.4
    NUM_MATCHED = 6

    def __init__(self):
        #cv2.namedWindow('frame')
        #cv2.namedWindow('cvt_frame')
        #cv2.namedWindow('IR_camera')
        #cv2.namedWindow('binary_camera')
        #cv2.namedWindow('dilate_frame')
        #cv2.namedWindow('erode_frame')
        #cv2.namedWindow('contours')
        cv2.namedWindow('rectangle')

    def _preprocess_image(self, _frame):

        blurred_frame = cv2.GaussianBlur(_frame, ksize= (3, 3), sigmaX= 0, sigmaY= 0)

        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit= 1.3, tileGridSize= (5, 5))
        clahe_s = clahe.apply(s)
        hsv_merge = cv2.merge((h, clahe_s, v))
        cvt_frame = cv2.cvtColor(hsv_merge, cv2.COLOR_HSV2BGR)
        ir_mark = cv2.inRange(hsv_merge, (120, 40, 210), (150, 90, 255))
        ir_frame = cv2.bitwise_and(hsv, hsv, mask= ir_mark)
        ir_frame = cv2.cvtColor(ir_frame, cv2.COLOR_HSV2BGR)

        gray_frame = cv2.cvtColor(ir_frame, cv2.COLOR_BGR2GRAY)
        _, binary_frame = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY)
        kernel = np.ones((3,3), np.uint8)
        dilate_frame = cv2.dilate(binary_frame, kernel, iterations = 2)
        kernel = np.ones((2,2), np.uint8)
        erode_frame = cv2.erode(dilate_frame, kernel, iterations = 2)
        _, contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            if 250 < area:
                cv2.rectangle(erode_frame, pt1 = (x, y), pt2 = (x+w, y+h), color = (230, 255, 100), thickness = -1)

        _, contours, _ = cv2.findContours(erode_frame, mode= cv2.RETR_EXTERNAL, method= cv2.CHAIN_APPROX_SIMPLE)

        result = np.zeros((480, 640, 3), dtype= np.uint8)
        cv2.drawContours(result, contours= contours, contourIdx= -1, color= (255, 255, 255), thickness= 0)

        #cv2.imshow("frame", _frame)
        #cv2.imshow("cvt_frame", cvt_frame)
        #cv2.imshow("IR_camera", ir_frame)
        #cv2.imshow("binary_camera", binary_frame)
        #cv2.imshow("erode_frame", erode_frame)
        #cv2.imshow("dilate_frame", dilate_frame)
        #cv2.imshow("contours", result)

        return result, contours

    def _select_ROI(self, _preprocessed_image, _contours):
        ROI_contoures = []

        for contour in _contours:
            x, y, w, h = cv2.boundingRect(contour)
            y_center_point = y + (w/2)
            x_center_point = x + (h/2)
            ratio = round(float(w)/float(h), 1)

            if self.MIN_WIDTH < w < self.MAX_WIDTH and self.MIN_HEIGHT < h < self.MAX_HEIGHT and self.MIN_RATIO < ratio < self.MAX_RATIO and self.MIN_POINT < y_center_point < self.MAX_POINT:
                cv2.rectangle(_preprocessed_image, pt1 = (x, y), pt2 = (x+w, y+h), color = (100, 255, 100), thickness = 1)

                ROI_contoures.append({
                    #'x_cp' : x_center_point, 'y_cp' : y_center_point, 'w' : w, 'h' : h, 'x' : x, 'y' : y
                    'x' : x, 'y' : y, 'w' : w, 'h' : h, 'x_cp' : x_center_point, 'y_cp' : y_center_point, 'w/h ratio' : ratio, 'contour' : contour
                })

        #print("ROI_contoures: ", ROI_contoures)
        cv2.imshow("rectangle", _preprocessed_image)

        return ROI_contoures

    def _match_contours(self, _ROI_contoures):
        cnt = 0
        sorted_upper_contours_list = []
        sorted_lower_contours_list = []
        sorted_possible_contoures = []
        matched_contours = []
        matched_contours_result = []
        
        if len(_ROI_contoures) != 6:
            print("The number of markers is wrong! marker: ", len(_ROI_contoures))
            return matched_contours_result
            
        _ROI_contoures = sorted(_ROI_contoures, key = (lambda y : y['y_cp']))

        sorted_upper_contours_list = _ROI_contoures[0:3]
        sorted_lower_contours_list = _ROI_contoures[3:6]

        sorted_upper_contours_list = sorted(sorted_upper_contours_list, key = (lambda x : x['x_cp']))
        sorted_lower_contours_list = sorted(sorted_lower_contours_list, key = (lambda x : x['x_cp']))

        for d1 in sorted_upper_contours_list:
            d1['idx'] = cnt
            cnt += 1
            sorted_possible_contoures.append(d1)

        for d2 in sorted_lower_contours_list:
            d2['idx'] = cnt
            cnt += 1
            sorted_possible_contoures.append(d2)

        #print("sorted_upper_contours_list: ", sorted_upper_contours_list)
        #print("sorted_lower_contours_list: ", sorted_lower_contours_list)
        #print("sorted_possible_contoures", sorted_possible_contoures)

        for d3 in sorted_possible_contoures:
            matched_contours = []
            for d4 in sorted_possible_contoures:
                if d3['idx'] == d4['idx']:
                    continue

                area_diff = abs(round(float(d3['w'] * d3['h'] - d4['w'] * d4['h']) / (d3['w'] * d3['h']), 1))
                #print(area_diff)

                if  area_diff < self.MAX_AREA_DIFF:
                    matched_contours.append(d4)

            matched_contours.append(d3)

            if len(matched_contours) != self.NUM_MATCHED:
                continue

            matched_contours_result.append(matched_contours)

            unmatched_contours = []
            for d5 in sorted_possible_contoures:
                if d5 not in matched_contours:
                    unmatched_contours.append(d5)

            try:
                unmatched_contour = np.take(self.sorted_possible_contoures, unmatched_contours)
                #recursive
                recursive_contour_list = self.find_chars(unmatched_contour)
                for d6 in recursive_contour_list:
                    matched_contours_result.append(d6)
            except:
                pass

            matched_contours_result = matched_contours_result[0]
            matched_contours_result = sorted(matched_contours_result, key = (lambda idx : idx['idx']))
            #print("matched_contours_result", matched_contours_result, end = '\n')

            break

        return matched_contours_result

    def _calculation_position(self, _mateched_contours):
        _diff_ratio = 0.0
        _area_ratio = 0.0
        _left_area = 0
        _right_area = 0
        _total_distance = 0
        _total_area = 0
        _center_check = "-"
        _robot_position = "-"

        if _mateched_contours == []:
            return _total_distance, _total_area, _diff_ratio, _center_check, _robot_position

        marker_center_point_x = (_mateched_contours[1]['x_cp'] + _mateched_contours[4]['x_cp'])/2
        
        if (320 - 3) < marker_center_point_x < (320 + 3):
            _center_check = 'CENTER'
        elif marker_center_point_x <= (320 - 3):
            _center_check = 'LEFT'
        elif marker_center_point_x >= (320 + 3):
            _center_check = 'RIGHT'
        elif marker_center_point_x == 0:
            _center_check = '-'
        else:
            _center_check = '-'

        #print('marker_center_point_x: ', marker_center_point_x)
        #print('_center_check: ', _center_check)

        _left_distance = _mateched_contours[1]['x_cp'] - _mateched_contours[0]['x_cp']
        _right_distance = _mateched_contours[2]['x_cp'] - _mateched_contours[1]['x_cp']
        _total_distance = _left_distance + _right_distance
        _left_area = (_left_distance) * (_mateched_contours[3]['y_cp'] - _mateched_contours[0]['y_cp'])
        _right_area = (_right_distance) * (_mateched_contours[5]['y_cp'] - _mateched_contours[2]['y_cp'])
        _total_area = _left_area + _right_area

        if _left_area > _right_area:
            _diff_ratio = round(float(_right_area)/float(_left_area), 2)
            if (_left_distance - _right_distance) <= 5:
                _robot_position = 'CENTER'
            else:
                _robot_position = 'LEFT'

        elif _right_area > _left_area:
            _diff_ratio = round(float(_left_area)/float(_right_area), 2)
            if (_right_distance - _left_distance) <= 5:
                _robot_position = 'CENTER'
            else:
                _robot_position = 'RIGHT'

        else:
            _diff_ratio = 1
            _robot_position = 'CENTER'
 
        #print("_center_check: ", _center_check)
        #print("_robot_position: ", _robot_position)
        #print("total_area: %d, diff_ratio: %f" %(_total_area, _diff_ratio))
        #print("left_area: %d, right_area: %d, total_area: %d, diff_ratio: %f" %(_left_area, _right_area, _total_area, _diff_ratio))

        return _total_distance, _total_area, _diff_ratio, _center_check, _robot_position

    def _calculation_distance(self, _total_distance, _total_area, _diff_ratio):
        _degree = 0.0
        _distance = 0.0
        _target_distance = 0.0

        if _total_area == 0:
            return _degree, _distance, _target_distance

        _degree = round(-36.83 * (_diff_ratio * _diff_ratio) + 118.45 * _diff_ratio + 8.28, 2)
        _distance = round(12222 * math.pow((_total_area + 500), -0.543), 2)

        radian = math.radians(_degree)
        _target_distance = round(_distance * (1/(math.tan(radian))), 2)

        #print("_degree: %f, _distance: %f, _target_distance: %f" %(_degree, _distance, _target_distance))

        return _degree, _distance, _target_distance

    def image_processing(self):
        ret, frame = self.capture.read()
        #print(ret)

        if ret == True:
            preprocessed_image, contours = self._preprocess_image(frame)
            ROI_contoures = self._select_ROI(preprocessed_image, contours)
            matched_contours = self._match_contours(ROI_contoures)
            total_distance, total_area, diff_ratio, self.center_check, self.robot_position = self._calculation_position(matched_contours)
            self.degree, self.distance, self.target_distance = self._calculation_distance(total_distance, total_area, diff_ratio)
        else:
            print("can't open video!!")
            self.degree = 0.0
            self.distance = 0.0
            self.target_distance = 0.0
            self.center_check = "-"
            self.robot_position = "-"

    def release(self):
        self.capture.release()
        self.degree = 0.0
        self.distance = 0.0
        self.target_distance = 0.0
        self.center_check = "-"
        self.robot_position = "-"



    def finish(self):
        self.capture.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    recog = recongnition()

    while True:
        recog.image_processing()

        #print("FPS: %f, Delay: %dms" %(recog.fps, recog.delay))

        if cv2.waitKey(recog.delay + 10) == ord('q'):
            break

    recog.finish()



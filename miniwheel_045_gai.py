#!/usr/bin/env python3.8
# coding=utf-8
import os
import sys

path = os.path.abspath(".")
# 核心
sys.path.insert(0,path + "/src/serial_pkg/scripts")
import time
import rospy
import numpy as np
import cv2
import ctypes
import serial
import struct 
import threading
from std_msgs.msg import UInt8, Int16
from geometry_msgs.msg import Twist
from myyolo import YoLov5TRT, plot_one_box


WHITE =255
BLACK =0

# HSV颜色提取阈值设置
#早上
# color_dist = {'red': {'L': np.array([120, 100, 46]), 'H': np.array([180, 255, 255])},
#               'green': {'L': np.array([40, 115, 113]), 'H': np.array([90, 255, 255])},
#               'red_line': {'L': np.array([135, 55, 100]), 'H': np.array([180, 255, 255])}, 
#               'yellow_line': {'L': np.array([26, 43, 83]), 'H': np.array([90, 255, 255])}
#              }

#晚上
color_dist = {'red': {'L': np.array([156, 160, 70]), 'H': np.array([180, 255, 255])},
              'green': {'L': np.array([35, 125, 90]), 'H': np.array([77, 255, 255])},
              'red_line': {'L': np.array([120, 43, 120]), 'H': np.array([180, 255, 255])}, 
              'yellow_line': {'L': np.array([26, 43, 83]), 'H': np.array([90, 255, 255])}
             }
# 目标检测标签
categories = ["safe_attention", "branch", "left_turn", "right_turn", "rate_limiting_off", "rate_limiting_on", "sidewalk"]

# 识别标志物标志
flag_turn_left  = False
flag_turn_right = True
flag_side_walk  = False
flag_limit_on   = False
flag_limit_off  = False
flag_branch     = False
flag_attention  = False

flag_red_light = False
flag_green_light = False

# 时间辅助
flag_start = False
car_speed_mode = 0 #0:正常速度 0.35  1：减速 0.25  2：加速0.45
flag_line_mode = 0
find_jiaodian = 0 #1:左转找 2：右转找 3:警告
flag_side_walk_once=0
time_count = 0
start_time = 0
flag_limit_once = 0
event_state = 0


# 霍夫变换参数
rho = 1
theta = np.pi / 180
threshold = 15
min_line_len = 20  # 线的最小长度
max_line_gap = 40  # 线段之间最大允许间隙

#老学长的斜率
# slope_threshold_min = 0.2  # 斜率检测参数
# slope_threshold_max = 10

#符哥的斜率
slope_threshold_min = 0.4  # 斜率检测参数
slope_threshold_max = 5
distance_threshold = 5  # 间距参数
exc_time = [-1.0, -1.0]


com=serial.Serial("/dev/ttyACM0",115200)

def send_data(cx,cz):
    action = bytearray([0x7B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D])
    cx_transition = (int)(cx * 100)
    cz_transition = (int)(cz * 100)
    print("VX:", cx_transition)
    print("V_Z:", cz_transition)
    action[1] = (cx_transition>>8) & 0xFF
    action[2] = cx_transition & 0xFF
    action[3] = (cz_transition>>8) & 0xFF
    action[4] = cz_transition & 0xFF

    action[5] = action[1]^action[2]^action[3]^action[4]

    for byte in action:
        com.write(byte.to_bytes(1, 'big'))


def find_traffic_light(image, color):
    """
    对红绿灯进行HSV处理

    :param image: 包含红绿灯的图像
    :param color: 需要提取的颜色
    :return: 红绿灯的坐标中心和面积大小
    """
    # 中值模糊
    blur_frame = cv2.medianBlur(image, 7)
    hsv_image = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)

    frame_binary = cv2.inRange(hsv_image, color_dist[color]['L'], color_dist[color]['H'])  # 使用数组进行图像二值化
    # 创建膨胀腐蚀核并进行膨胀腐蚀
    kernel = (3, 3)
    frame_binary_DE = cv2.erode(frame_binary, kernel, iterations=1)
    frame_binary_DE = cv2.dilate(frame_binary_DE, kernel, iterations=2)
    # cv2.imshow('frame_binary_DE', frame_binary_DE)

    contours = cv2.findContours(frame_binary_DE, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    len_cont = len(contours)
    max_cont_area = 0
    center_xy = [0, 0]
    if len_cont > 0:
        c = max(contours, key=cv2.contourArea)
        max_cont_area = cv2.contourArea(c)
        xy_mean = np.mean(c, 0)
        center_xy[0] = int(xy_mean[0][0])
        center_xy[1] = int(xy_mean[0][1])

    return max_cont_area, center_xy

def red_line(image, color):
    """
    对斜坡处红色色块进行HSV处理

    :param image: 输入的图像
    :param color: 需要提取的颜色
    :return: 红色色块的总的面积
    """
    gs_frame = cv2.GaussianBlur(image, (5, 5), 0)  # 高斯模糊
    hsv_image = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)

    frame_binary = cv2.inRange(hsv_image, color_dist[color]['L'], color_dist[color]['H'])  # 使用数组进行图像二值化
    # 创建膨胀腐蚀核并进行膨胀腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    frame_binary_DE = cv2.erode(frame_binary, kernel, iterations=1)
    frame_binary_DE = cv2.dilate(frame_binary_DE, kernel, iterations=2)
    # cv2.imshow('frame_binary_DE', frame_binary_DE)

    # 查找轮廓
    contours = cv2.findContours(frame_binary_DE, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    len_cont = len(contours)

    all_cont_area = 0
    if len_cont > 0:
        for c in contours:
            all_cont_area += cv2.contourArea(c)

    return all_cont_area


def get_max_area(boxs, result_classids):
    """
    对目标检测识别的结果进行过滤, 得到面积最大的那个检测目标

    :param boxs: bboxes列表
    :param result_classids: 对应的class_ids列表
    :return: class_id, 最大面积的检测目标的面积和对应的矩形框[xmin, ymin, xmax, ymax]
    """
    max_area = 0
    cls_id = 0
    _box = None
    for box, class_id in zip(boxs, result_classids):
        x1, y1, x2, y2 = box
        area = (x2 - x1) * (y2 - y1)
        if area > max_area:
            max_area = int(area)
            cls_id = int(class_id)
            _box = box

    return cls_id, max_area, _box


#*图像裁剪处理*
def image_sizeprocess(input_img):
    col = 320;	 # width
    row = 240;	 # height
    row_p = 120; # prospect120
    
    input_img = cv2.resize(input_img, (col, row), interpolation= cv2.INTER_AREA)
    x, y = 0, 120
    w, h = 320, 120
    input_img = input_img[y:y+h, x:x+w]
    input_img = cv2.resize(input_img, (300, 150), interpolation=cv2.INTER_LINEAR)
    return input_img


#*图像预处理*
def image_process(input_img, color):   
    """
    图像处理函数，返回一个二值化图像

    :param input_img: 输入裁剪好的图像
    :param color: 指定二值化的颜色
    :return: 处理好的二值化图像
    """
    input_img = cv2.GaussianBlur(input_img, (3, 3), 1, 1)   
    hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
    frame_process = cv2.inRange(hsv_img, color_dist[color]['L'], color_dist[color]['H'])
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    frame_process = cv2.dilate(frame_process,kernel,iterations=1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)) #ksize=3,3
    frame_process = cv2.erode(frame_process,kernel,iterations=1)

    return frame_process

def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
    """
    统计概率霍夫直线变换

    :param img: 输入的图像, 二值化图
    :param rho: 距离分辨率
    :param theta: 角度分辨率
    :param threshold: 一条直线的最小的曲线交点
    :param min_line_len: 最小线长阈值
    :param max_line_gap: 最大线距阈值
    :return: 得到的符合要求的直线
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold,
                            minLineLength=min_line_len, maxLineGap=max_line_gap)
    return lines


def choose_lines(lines, slope_threshold_min, slope_threshold_max):
    """
    划分左右车道

    :param lines: 输入的直线列表
    :param slope_threshold_min: 最小斜率阈值
    :param slope_threshold_max: 最大斜率阈值
    :return: 左车道线和右车道线
    """
    left_lines, right_lines = [], []

    for line in lines:
        for x1, y1, x2, y2 in line:
            if x1 != x2:
                k = float(y1 - y2) / (x1 - x2)
                if slope_threshold_max > abs(k) > slope_threshold_min:
                    if k < 0:
                        if ((x1 + x2) / 2) > 150:
                            continue
                        else:
                            left_lines.append(line)
                    else:
                        if ((x1 + x2) / 2) < 150:
                            continue
                        else:
                            right_lines.append(line)
                else:
                    continue

    return left_lines, right_lines

def select_points(rows, line_list, distance_threshold):
    max_count = 0
    xmin_points = [(x2 - x1) / (y2 - y1) * (rows - y1) + x1 for line in line_list for x1, y1, x2, y2 in line]
    standard_num = xmin_points[0]
    for i in range(0, len(xmin_points), 1):
        count = 0
        for points in xmin_points:
            if xmin_points[i] - distance_threshold < points < xmin_points[i] + distance_threshold:
                count = count + 1
        if count > max_count:
            standard_num = xmin_points[i]
            max_count = count
    list_del_index = []
    for i in range(0, len(xmin_points), 1):
        if xmin_points[i] < standard_num - distance_threshold or xmin_points[i] > standard_num + distance_threshold:
            list_del_index.append(i)
    counter = 0
    if len(list_del_index) > 0:
        for index in list_del_index:
            index = index - counter
            line_list.pop(index)
            counter += 1

# def select_points(rows, line_list, distance_threshold):
#     """
#     在图像中选出最密集的直线，删除零散的直线

#     :param rows: 输入的图像的行数阈值
#     :param line_list: 直线列表
#     :param distance_threshold: 间距阈值
#     :return: 筛选后的车道线
#     """
#     max_count = 0
#     xmin_points = [(x2 - x1) / (y2 - y1) * (rows - y1) + x1 for line in line_list for x1, y1, x2, y2 in line]
#     standard_num = xmin_points[0]
#     for i in range(0, len(xmin_points), 1):
#         count = 0
#         for points in xmin_points:
#             if xmin_points[i] - distance_threshold < points < xmin_points[i] + distance_threshold:
#                 count = count + 1
#         if count > max_count:
#             standard_num = xmin_points[i]
#             max_count = count
#     list_del_index = []
#     for i in range(0, len(xmin_points), 1):
#         if xmin_points[i] < standard_num - distance_threshold or xmin_points[i] > standard_num + distance_threshold:
#             list_del_index.append(i)
#     counter = 0
#     if len(list_del_index) > 0:
#         for index in list_del_index:
#             index = index - counter
#             line_list.pop(index)
#             counter += 1

def least_squares_fit(point_list, ymin, ymax):
    """
    最小二乘法拟合

    :param point_list: 输入的直线点值
    :param ymin: 给定区域的y最小值
    :param ymax: 给定区域的y最大值
    :return: 拟合后的直线的两端的点值
    """
    x = [p[0] for p in point_list]
    y = [p[1] for p in point_list]

    # polyfit第三个参数为拟合多项式的阶数，所以1代表线性
    fit = np.polyfit(y, x, 1)
    fit_fn = np.poly1d(fit)  # 获取拟合的结果

    xmin = int(fit_fn(ymin))
    xmax = int(fit_fn(ymax))

    return [(xmin, ymin), (xmax, ymax)]


def get_lane_info(input_roi_edges):
    """
    输入为待检测图片, 经过处理后获取左右车道线

    :param input_roi_edges: 输入的图像
    :return: 左侧直线结果、右侧直线结果
    """
    h = input_roi_edges.shape[0]-1
    lines = hough_lines(input_roi_edges, rho, theta, threshold, min_line_len, max_line_gap)
    left_results, right_results = [], []

    if lines is not None:
        left_lines, right_lines = choose_lines(lines, slope_threshold_min, slope_threshold_max)
        if len(left_lines) > 0:
            select_points(h, left_lines, distance_threshold)

            left_points = [(x1, y1) for line in left_lines for x1, y1, x2, y2 in line]
            left_points = left_points + [(x2, y2) for line in left_lines for x1, y1, x2, y2 in line]
            left_results = least_squares_fit(left_points, 0, h)

        if len(right_lines) > 0:
            select_points(h, right_lines, distance_threshold)

            right_points = [(x1, y1) for line in right_lines for x1, y1, x2, y2 in line]
            right_points = right_points + [(x2, y2) for line in right_lines for x1, y1, x2, y2 in line]
            right_results = least_squares_fit(right_points, 0, h)

    return left_results, right_results

def get_turn_error_hp(input_cv_img, left_results, right_results):
    """
    获取车道线的偏差值

    :param input_cv_img: 输入的图像
    :param left_results: 左车道线
    :param right_results: 右车道线
    :return: 车道线偏差值
    """
    global conner_flag
    global cnt_sensor
    global start_cnt
    global start_x
    global Last_turn_error

    height = input_cv_img.shape[0]
    width = input_cv_img.shape[1]

    # 两边都有线
    if len(left_results) > 0 and len(right_results) > 0:
        X1 = (left_results[0][0] + right_results[0][0]) / 2
        X2 = (left_results[1][0] + right_results[1][0]) / 2
        Turn_error = (X1 + X2) / 2

    # 开始部分进行补线
    # elif len(right_results) == 0 and len(left_results) != 0 and cnt_sensor == 10:
    #     X = left_results[1][0] + (start_x / start_cnt) * 0.5
    #     Turn_error = (width / 2 - X) * 0.9

    # # 只有右线-----左转
    # elif len(left_results) == 0 and len(right_results) != 0:
    #     X_top = right_results[0][0]
    #     X_bottom = (height - right_results[0][1]) * (right_results[0][0] - right_results[1][0]) / (
    #             right_results[0][1] - right_results[1][1]) + right_results[0][0]
    #     X = (X_top - 190) / 2 + (X_bottom - 260) / 2
    #     Turn_error = (width / 2 - X)  # Turn left

    # 只有左线-----右转
    # if len(right_results) == 0 and len(left_results) != 0:
    #     X_top = left_results[0][0]
    #     X_bottom = (height - left_results[0][1]) * (left_results[0][0] - left_results[1][0]) / (
    #             left_results[0][1] - left_results[1][1]) + left_results[0][0]
    #     X = (X_top + 130) / 2 + (X_bottom + 220) / 2
    #     Turn_error = (width / 2 - 15 - X) * 0.91  # negative # 0.91

    if len(right_results) == 0 and len(left_results) != 0:
        Turn_error = (left_results[0][0] + left_results[1][0]) / 2 + 190

    # elif flag_speed == False and flag_turn_left == True and len(right_results) == 0 and len(left_results) == 0:
    #     Turn_error = Last_turn_error

    # else:
    #     Turn_error = 0

    # # 第一个转角
    # if flag_side_walk:
    #     Turn_error = Turn_error - 3

    # if flag_speed == False and flag_turn_left == True:
    #     Last_turn_error = Turn_error

    return Turn_error

def limit_val(a,max,min):
    if a > max:
        return max
    elif a < min:
        return min
    else:
        return a

#先塞入的是巡线的起点，巡线数组为倒序
def get_line(in_frame_process):
    """
    巡线函数, 完成巡线

    :param in_frame_process: 输入处理好的图像
    :return: error 偏差
    """
    global find_jiaodian
    global flag_side_walk
    global flag_line_mode

    find_mid=150  
    find_buttom=150

    middle_line = []
    left_line = []
    right_line = []
    L_find_state = []
    R_find_state = []

    L_start_point=0
    R_start_point=0

    road_width=270

    #第70-79行路宽
    width_list = [184, 184, 187, 188, 188,
                  191, 192, 192, 195, 196]

    offset = 0
    lose_left_line_state = 0#lose count init
    lose_right_line_state = 0
    
    for i in range(130, -1, -1):     #0-129行作为扫线行; 130,129,128作为起点寻找行
        middle_line.append(150)
        left_line.append(3)
        right_line.append(297)   
        L_find_state.append(0)
        R_find_state.append(0)

    middle_line = np.array(middle_line)
    left_line = np.array(left_line)
    right_line = np.array(right_line)
    L_find_state = np.array(L_find_state)
    R_find_state = np.array(R_find_state)

    for j in range(find_buttom-20, find_buttom-22-1, -1):  #130,129,128
        #找左线
        for i in range(find_mid-1, 3-1, -1):
            if in_frame_process[j,i+1]== WHITE and in_frame_process[j, i-1]==WHITE:
                left_line[j] = i
                L_find_state[j] = 1 #找到线
                break
            else:
                L_find_state[j] = 0 #lose 线
                left_line[j] = find_mid-1

        #找右线
        for i in range(find_mid+1, 297+1, 1):
            if in_frame_process[j,i-1]==WHITE and in_frame_process[j,i+1]==WHITE:
                right_line[j] = i
                R_find_state[j] = 1
                break
            else:
                R_find_state[j] = 0#lose 线
                right_line[j] = find_mid+1

    #处理左线起点
    if L_find_state[130]==1 and L_find_state[129]==1 and L_find_state[128]==1 :
        L_start_point = (int)((left_line[130] + left_line[129] + left_line[128]) / 3) + 30
    else:
        L_start_point = find_mid

    #处理右线起点
    if R_find_state[130]==1 and R_find_state[129]==1 and R_find_state[128]==1 :
        R_start_point = (int)((right_line[130] + right_line[129] + right_line[128]) / 3) - 30
    else:
        R_start_point = find_mid       


    L_start_point = limit_val(L_start_point,297 , 3)
    R_start_point = limit_val(R_start_point, 297, 3)
    L_start_point = limit_val(L_start_point,R_start_point , 3)
    R_start_point = limit_val(R_start_point, 297, L_start_point)
    # print("L", L_start_point, "R", R_start_point)
#-------------------------------------------------------------------------------
    if flag_line_mode == 0:
        for j in range(find_buttom-21, -1, -1):  #129
            #找左线
            for i in range(L_start_point, 3-1, -2):
                # print(L_start_point)#yuejie
                if in_frame_process[j,i+1]== WHITE and in_frame_process[j, i-1]==WHITE:
                    left_line[j] = i
                    L_find_state[j] = 1 #找到线
                    break
                else:
                    L_find_state[j] = 0 #lose 线
                    left_line[j] = 3

            if L_find_state[j] == 0:      
                lose_left_line_state+=1
        
            L_start_point = left_line[j] + 30
                #找右线
            for i in range(R_start_point, 297+1, 1):
                # print(R_start_point)#yuejie
                if in_frame_process[j,i-1]==WHITE and in_frame_process[j,i+1]==WHITE:
                    right_line[j] = i
                    R_find_state[j] = 1
                    break
                else:
                    R_find_state[j] = 0
                    right_line[j] = 297
   
            if R_find_state[j] == 0:      
                lose_right_line_state+=1
        
            R_start_point = right_line[j] - 30

            L_start_point = limit_val(L_start_point,297 , 3)
            R_start_point = limit_val(R_start_point, 297, 3)
            L_start_point = limit_val(L_start_point,R_start_point , 3)
            R_start_point = limit_val(R_start_point, 297, L_start_point)

    elif flag_line_mode == 1:
        for j in range(find_buttom-21, -1, -1):  #129
            #找左线
            for i in range(L_start_point, 3-1, -2):
                # print(L_start_point)#yuejie
                if in_frame_process[j,i+1]== WHITE and in_frame_process[j, i-1]==WHITE:
                    left_line[j] = i
                    L_find_state[j] = 1 #找到线
                    break
                else:
                    L_find_state[j] = 0 #lose 线
                    left_line[j] = 3

            if L_find_state[j] == 0:      
                lose_left_line_state+=1
        
                #找右线
            for i in range(R_start_point, 297+1, 1):
                # print(R_start_point)#yuejie
                if in_frame_process[j,i-1]==WHITE and in_frame_process[j,i+1]==WHITE:
                    right_line[j] = i
                    R_find_state[j] = 1
                    break
                else:
                    R_find_state[j] = 0
                    right_line[j] = 297

            if R_find_state[j] == 0:      
                lose_right_line_state+=1
        
            if j != 129:
                limit_val(left_line[j], left_line[j+1]+10, left_line[j+1]-10)
                limit_val(right_line[j], right_line[j+1]+10, right_line[j+1]-10)

            limit_val(left_line[j], left_line[j+1]+10, 3)
            limit_val(right_line[j], 297, right_line[j+1]-10)


            L_start_point = left_line[j] + 30
            R_start_point = right_line[j] - 30

            L_start_point = limit_val(L_start_point,297 , 3)
            R_start_point = limit_val(R_start_point, 297, 3)
            L_start_point = limit_val(L_start_point,R_start_point , 3)
            R_start_point = limit_val(R_start_point, 297, L_start_point)

    elif flag_line_mode == 2:
        for j in range(find_buttom-21, -1, -1):  #129
            #找左线
            for i in range(L_start_point, 3-1, -2):
                # print(L_start_point)#yuejie
                if in_frame_process[j,i+1]== WHITE and in_frame_process[j, i-1]==WHITE:
                    left_line[j] = i
                    L_find_state[j] = 1 #找到线
                    break
                else:
                    L_find_state[j] = 0 #lose 线
                    left_line[j] = 3

            if L_find_state[j] == 0:      
                lose_left_line_state+=1
        
            L_start_point = left_line[j] + 30
            L_start_point = limit_val(L_start_point, 297, 3)

        #根据右线更新左线
        for j in range(0,10):
            right_line[j] = left_line[70+j] + width_list[j]

    elif flag_line_mode == 3:
        for j in range(find_buttom-21, -1, -1):  #129         
                #找右线
            for i in range(R_start_point, 297+1, 1):
                # print(R_start_point)#yuejie
                if in_frame_process[j,i-1]==WHITE and in_frame_process[j,i+1]==WHITE:
                    right_line[j] = i
                    R_find_state[j] = 1
                    break
                else:
                    R_find_state[j] = 0
                    right_line[j] = 297

            if R_find_state[j] == 0:      
                lose_right_line_state+=1

            R_start_point = right_line[j] - 30
            R_start_point = limit_val(R_start_point, 297, 3)

        #根据右线更新左线
        for j in range(0,10):
            left_line[j] = right_line[70+j] - width_list[j]
        
            
          
    #jiaodian 左转
    if flag_side_walk==True and find_jiaodian == 1 and lose_left_line_state>50 and lose_right_line_state>50:
        k =0 
        jiaodian_y = 0
        jiaodian_x = 0
        print("jiaodian")
        for j in range(69, 9, -1):
            for i in range(100 ,find_mid+50):
                if in_frame_process[j,i]==WHITE and in_frame_process[j-3,i]==WHITE and in_frame_process[j-10,i]==BLACK:
                    jiaodian_y = j
                    jiaodian_x = i
                    right_line[jiaodian_y] = jiaodian_x
                    if not R_find_state[find_buttom-21]:
                        right_line[find_buttom-21] = left_line[find_buttom-21] + road_width

                    k =  (jiaodian_x-(right_line[find_buttom-21])) / (jiaodian_y-(find_buttom-21)) 
			
					#fix line
                    for row in range(find_buttom-22, jiaodian_y,-1):
                        right_line[row] = (row-(find_buttom-21))*k + right_line[find_buttom-21]
                    break

            if jiaodian_y != 0:
                jiaodian_y = 0
                break
        #jiaodian 右转
    elif flag_side_walk==True and find_jiaodian == 2 and lose_left_line_state>50 and lose_right_line_state>50:
        k =0 
        jiaodian_y = 0
        jiaodian_x = 0
        print("jiaodian")
        for j in range(69, 9, -1):
            for i in range(find_mid+50, 100, -1):
                if in_frame_process[j,i]==WHITE and in_frame_process[j-3,i]==WHITE and in_frame_process[j-10,i]==BLACK:
                    jiaodian_y = j
                    jiaodian_x = i
                    left_line[jiaodian_y] = jiaodian_x
                    if not L_find_state[find_buttom-21]:
                        left_line[find_buttom-21] = right_line[find_buttom-21] - road_width

                    k =  (jiaodian_x-(left_line[find_buttom-21])) / (jiaodian_y-(find_buttom-21)) 
			
					#fix line
                    for row in range(find_buttom-22, jiaodian_y,-1):
                        left_line[row] = (row-(find_buttom-21))*k + left_line[find_buttom-21]
                    break

            if jiaodian_y != 0:
                jiaodian_y = 0
                break
    elif flag_attention == True and find_jiaodian == 3 and lose_left_line_state>100 and lose_right_line_state>10:
        k = 0 
        jiaodian_y = 0
        jiaodian_x = 0
        print("jiaodian")
        for j in range(50, 3, -1):
            for i in range(20, find_mid, 1):
                if in_frame_process[j,i-1]==WHITE and in_frame_process[j,i+1]==WHITE and in_frame_process[j,i-20]==BLACK and in_frame_process[j,i+20]==BLACK:
                    jiaodian_y = j
                    jiaodian_x = i
                    right_line[jiaodian_y] = jiaodian_x
                    # if R_find_state[find_buttom-21]:
                    k =  (jiaodian_x-(right_line[find_buttom-21])) / (jiaodian_y-(find_buttom-21))                
                    #fix line
                    for row in range(find_buttom-22, jiaodian_y,-1):
                        right_line[row] = (row-(find_buttom-21))*k + right_line[find_buttom-21]
                        left_line[row] = right_line[row] - road_width
                    break

            if jiaodian_y != 0:
                jiaodian_y = 0 
                break
    # elif flag_attention == True and find_jiaodian == 3 and lose_left_line_state>100 and lose_right_line_state>10:
    #     k = 0 
    #     jiaodian_y = 0
    #     jiaodian_x = 0
    #     print("jiaodian")
    #     for j in range(50, 3, -1):
    #         for i in range(20, find_mid, 1):
    #             if in_frame_process[j,i]==WHITE and in_frame_process[j-2,i+5]==WHITE and in_frame_process[j,i-20]==BLACK:
    #                 jiaodian_y = j
    #                 jiaodian_x = i
    #                 right_line[jiaodian_y] = jiaodian_x
    #                 if R_find_state[find_buttom-21]:
    #                     k =  (jiaodian_x-(right_line[find_buttom-21])) / (jiaodian_y-(find_buttom-21))                
    #                     #fix line
    #                     for row in range(find_buttom-22, jiaodian_y,-1):
    #                         right_line[row] = (row-(find_buttom-21))*k + right_line[find_buttom-21]
    #                         left_line[row] = right_line[row] - road_width
    #                     break

    #         if jiaodian_y != 0:
    #             jiaodian_y = 0 
    #             break

    for j in range(find_buttom-21, -1, -1):
        middle_line[j] = int((left_line[j] + right_line[j]) / 2 )

    return left_line, right_line, middle_line, lose_left_line_state, lose_right_line_state


def get_error(in_middle_line):
    """
    求道路偏差函数, 完成巡线后求取中线偏差

    :param in_middle_line: 输入处理好后的中线数组
    :return: 偏差值，浮点数
    """
    target_midline = 150
    prospect_line = 70
    end_line = 80
    error = 0.0
    for j in range(prospect_line, end_line):
        error += target_midline - in_middle_line[j]

    error = float(error / (end_line - prospect_line))

    return error


def visual_ctrl(in_image, twist, cls_id, area):
    """
    巡线函数, 完成巡线和特定任务的执行

    :param in_image: 输入缩放好的图像
    :param twist: 速度变量
    :param cls: 类别id
    :param max_area: 检测目标的面积
    :return: 速度值, 包括行驶速度和角速度
    """
    global flag_turn_left
    global flag_turn_right 
    global flag_side_walk  
    global flag_limit_on   
    global flag_limit_off 
    global flag_branch     
    global flag_attention 
    global flag_green_light 
    global time_count 
    global flag_start
    global car_speed_mode
    global flag_line_mode
    global find_jiaodian
    global flag_side_walk_once
    global flag_limit_once
    global event_state
    global start_time
    # start_time = time.time()

                                                        # 处理图像，输出对应黄色线的二值化图，使用模式0
    if flag_line_mode==0 or flag_line_mode == 2:
        line_image = image_process(in_image,'yellow_line')
    elif flag_line_mode==3:                            # 处理图像，输出对应红色线的二值化图，使用模式2
        line_image = image_process(in_image,'red_line')

    # cv2.imshow("line", line_image)
    # if flag_limit_on == False:
    left_line, right_line, middle_line, lose_left_line_state, lose_right_line_state = get_line(line_image)
    turn_error = get_error(middle_line)
    # print('lose',lose_left_line_state, 'and', lose_right_line_state)

    for j in range(150-21, -1, -1):
    # # for j in range(150-21, 128, -1):
        cv2.circle(in_image, (left_line[j],j), 1, (255, 0, 0),-1)#blue
        cv2.circle(in_image, (right_line[j],j),1, (0, 255, 0),-1)#green
        cv2.circle(in_image, (middle_line[j],j),1, (0, 255, 255),-1)#yellow

    # 根据二值化的图像获取左右车道线
    # elif flag_limit_on == True:
    #     left_results, right_results = get_lane_info(line_image)
    #     turn_error = get_turn_error_hp(line_image, left_results, right_results)
        # cv2.line(in_image, (left_results[0][0], left_results[0][1]), (left_results[1][0], left_results[1][1]), (0, 0, 255), 2)
        # cv2.line(in_image, (right_results[0][0], right_results[0][1]), (right_results[1][0], right_results[1][1]), (0, 0, 255), 2)

    
    cv2.imshow("line", in_image)
    # end_time = time.time()
    # print((end_time-start_time)*1000)
    # for j in range(70, 80):
    #     width = right_line[j] - left_line[j]
    #     print('第', j, '行:',width)

    
    print('turnerror', turn_error)
    if twist.linear.x == 0.25:
        twist.angular.z = 0.008* turn_error
    elif twist.linear.x == 0.2:
        twist.angular.z = 0.008* turn_error
    elif twist.linear.x == 0.3:
        twist.angular.z = 0.009* turn_error   
    elif twist.linear.x == 0.35:
        twist.angular.z = 0.012* turn_error  
    elif twist.linear.x == 0.45:
        twist.angular.z = 0.014* turn_error  
    elif twist.linear.x == 0.55:
        twist.angular.z = 0.016* turn_error  
    elif twist.linear.x == 0.6:
        twist.angular.z = 0.001* turn_error 

    twist.angular.z = limit_val(twist.angular.z,5.0,-5.0)

    if car_speed_mode == 0:  #给出基础速度
        twist.linear.x = 0.45
    elif car_speed_mode == 1:
        twist.linear.x = 0.3
    elif car_speed_mode == 2: 
        twist.linear.x = 0.55 
    elif car_speed_mode == 3: 
        twist.linear.x = 0.35 
    #处理完正常巡线，进行标识处理
    '''
            0               1           2           3               4                   5               6
    ["safe_attention", "branch", "left_turn", "right_turn", "rate_limiting_off", "rate_limiting_on", "sidewalk"]
    '''

    if flag_turn_left == True:
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1:
            end_time = time.time()
            time_count = end_time - start_time
            if time_count > 0 and time_count < 2:
                twist.linear.x = 0.25
                twist.angular.z = 0.28
            else:
                event_state = 2
        if event_state == 2:
            flag_turn_left = False
            find_jiaodian = 1
            event_state = 0
            time_count = 0
            
    if flag_turn_right == True:
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1:
            end_time = time.time()
            time_count = end_time - start_time
            if time_count > 0 and time_count < 2:
                twist.linear.x = 0.25
                twist.angular.z = -0.28
            else:
                event_state = 2
        if event_state == 2:
            flag_turn_right = False
            find_jiaodian = 2
            event_state = 0
            time_count = 0

    if (int(cls_id)==6 or int(cls_id)==5) and area>1500 and flag_side_walk_once==0:
        flag_side_walk_once = 1
        flag_side_walk = True

    if flag_side_walk:
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1 and area>6000:
            end_time = time.time()
            time_count = end_time - start_time
            if time_count > 0 and time_count < 5:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                event_state = 2
        if event_state == 2:
            flag_side_walk = False
            car_speed_mode = 2
            event_state = 0
            time_count = 0

    if int(cls_id==5) and area>7300 and flag_limit_once==0 and flag_side_walk==False and flag_side_walk_once == 1:
        flag_limit_on=True
        flag_limit_once = 1
        flag_line_mode = 2
        car_speed_mode = 1

    if flag_limit_on:
        #准备入弯
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1:
            end_time = time.time()
            time_count = end_time - start_time
            twist.angular.z = limit_val(twist.angular.z, -0.05 , -0.35)
            # if time_count > 0 and time_count < 2:
            #     twist.linear.x = 0.25
            #     twist.angular.z = -0.2
            # if time_count > 2 and time_count < 4:
            #     twist.linear.x = 0.25
            #     twist.angular.z = -0.3
            # elif time_count > 4:
            #     event_state = 2  #等待限速事件结束
            if time_count > 2.5:
                event_state = 2  #等待限速事件结束


    if int(cls_id==4) and area>4000 and event_state==2 :
        flag_limit_on = False 
        flag_limit_off = True
        flag_line_mode = 0
        car_speed_mode = 0
        event_state = 0
        time_count = 0
    
    if flag_limit_off: 
        # pass
        twist.angular.z = limit_val(twist.angular.z, 0 ,-0.5)


    if int(cls_id==1) and area>2400:
        flag_branch = True
        car_speed_mode = 3

    if flag_branch==True and flag_red_light == False and flag_green_light == False:
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1:
            end_time = time.time()
            time_count = end_time - start_time
            if time_count > 0 and time_count < 2:   #准备入岔道，矫正姿态
                twist.linear.x = 0.35
                twist.angular.z = -0.35
            else:
                event_state = 2
                flag_line_mode = 3
        if event_state == 2:
            end_time = time.time()
            time_count = end_time - start_time
            print(lose_right_line_state)
            print(time_count)
            if lose_right_line_state > 75 and time_count>6:
                flag_line_mode = 0
                flag_branch = False
                car_speed_mode = 2
                event_state = 0
                time_count = 0

    if car_speed_mode == 2:
        if (int(cls_id==0) and area>2500) or (int(cls_id==1) and area>2500):
            flag_attention = True

    if flag_attention == True:
        if event_state == 0:
            event_state = 1
            start_time = time.time()
        if event_state == 1:
            end_time = time.time()
            time_count = end_time - start_time
            if time_count > 0 and time_count < 3:   #看到警告，矫正姿态
                find_jiaodian = 3
            else:
                event_state = 2
        if event_state == 2:
            flag_attention = False
            find_jiaodian = 0
            event_state = 0
            time_count = 0
    # if flag_attention == True:
    #     if event_state == 0:
    #         event_state = 1
    #         start_time = time.time()
    #     if event_state == 1:
    #         end_time = time.time()
    #         time_count = end_time - start_time
    #         if time_count > 0 and time_count < 1:   #看到警告，矫正姿态
    #             twist.linear.x = 0.35
    #             twist.angular.z = 0.5
    #         else:
    #             event_state = 2
    #     if event_state == 2:
    #         flag_attention = False
    #         event_state = 0
    #         time_count = 0
                

    
    

    return twist


def main():
    # cv2.namedWindow("Frame", cv2.WINDOW_AUTOSIZE)

    global flag_turn_left
    global flag_turn_right 
    global flag_side_walk  
    global flag_limit_on   
    global flag_limit_off 
    global flag_branch     
    global flag_attention 
    global flag_red_light 
    global flag_green_light 
    global start_time

    cap = cv2.VideoCapture(0)
    width = 640  #定义摄像头获取图像宽度
    height = 480   #定义摄像头获取图像长度
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  #设置宽度
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  #设置长度

    # cap = cv2.VideoCapture("/home/jetson/Desktop/test/newsaidao_quancheng.mp4")

    event = threading.Event()
    twist = Twist()
    # # 创建速度发布话题
    # cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    # 加载yolo模型
    PLUGIN_LIBRARY = "/home/jetson/catkin_ws/src/serial_pkg/scripts/Orin_yolov5TRT2/build/libmyplugins.so"
    engine_file_path = "/home/jetson/catkin_ws/src/serial_pkg/scripts/Orin_yolov5TRT2/build/yolov5s.engine"
    ctypes.CDLL(PLUGIN_LIBRARY)
    yolov5_wrapper = YoLov5TRT(engine_file_path)
    
    # rate = rospy.Rate(50)
    # while not rospy.is_shutdown():
    while True:
        # start_time = time.time()
        ret, img = cap.read()
        if not ret:
            continue
        # # 缩放巡线图像, 提高计算效率
        image = image_sizeprocess(img) 

        
        # if speed_limit == True:
        #     line_image = image[120:360, :, :]  # 600x240
        #     line_image = cv2.resize(line_image, (300, 120), interpolation=cv2.INTER_LINEAR)
        # else:
        #     line_image = image[250:330, :, :]  # 640x80
        #     line_image = cv2.resize(line_image, (300, 40), interpolation=cv2.INTER_LINEAR)

        # if not change_frame:
        frame = img  # 448x224 首先裁剪右侧图像用于yolo推理
        # else:
        #     frame = image[:224, 96:320, :]  # 224x224 裁剪左侧图像用于yolo推理

        # yolov5-nano
        
            # 进行yolo推理
        result_boxes, result_scores, result_classid, use_time = yolov5_wrapper.infer(frame)

        cls_id = 10
        area = 0
        
        # 获取当前面积最大的检测目标
        cls_id, area, box = get_max_area(result_boxes, result_classid)
        if area > 1000:
            plot_one_box(box, frame, (0, 0, 255), categories[cls_id])
            print(area)

        cv2.imshow("Frame", frame)
        # 进行巡线
        twist = visual_ctrl(image, twist, cls_id, area)

        # send_data2(0.3, 0.0)
        # # light 红绿灯处理
        
        if flag_limit_off==True or flag_red_light==True:
            light_image = img
        # hsv = cv2.cvtColor(light_image, cv2.COLOR_BGR2HSV)
        # dst = cv2.inRange(hsv, color_dist['red']['L'], color_dist['red']['H'])
            green_area, green_center_xy = find_traffic_light(light_image, 'green')  # 180x200
            red_area, red_center_xy = find_traffic_light(light_image, 'red')  # 180x200
        # # twist.linear.x = 0.3
        # # twist.angular.z = 0.5
        # # if green_area > red_area and green_area > 30:
        # #         find_green_light = False

        # #         conner_flag = True
        #     # cv2.circle(light_image, (green_center_xy[0], green_center_xy[1]), 20, (0, 255, 0), 4, cv2.LINE_AA)
            print('green:', green_area)
            print('red:', red_area)
            cv2.circle(light_image, (green_center_xy[0], green_center_xy[1]), 20, (0, 255, 0), 4, cv2.LINE_AA)
            cv2.circle(light_image, (red_center_xy[0], red_center_xy[1]), 20, (0, 0, 255), 4, cv2.LINE_AA)
            #红灯事件预备
            if flag_green_light == False:
                if red_area > 2500 and red_area<4500:
                    flag_limit_off = False
                    flag_red_light = True
                    twist.linear.x = 0.8
                    twist.angular.z = limit_val(twist.angular.z, 0,0)
                #红灯事件执行
                elif red_area > 4500:
                    flag_green_light = True #等待绿灯
                    start_time = time.time()
            elif flag_green_light == True:
                end_time = time.time()
                if (end_time - start_time) < 10 and green_area < 4500:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif green_area > 4500:
                    flag_red_light = False #红灯结束，红灯标识关闭，不再识别红绿灯
                    flag_green_light = False

        # #     else:
        # #         twist.linear.x = 0.

            # cv2.imshow("Light", light_image)


        # # 订阅标志位和俯仰角
        # sensor_ = rospy.Subscriber("/SensorCount", UInt8, sensor_callback)
        # pitch_ = rospy.Subscriber("/Pitch", Int16, pitch_callback)

        # 发布速度指令话题
        send_data(twist.linear.x,twist.angular.z)
        # cmd_vel_pub.publish(twist)

        if (cv2.waitKey(1) & 0xFF) == ord('q'):
            break
        # end_time = time.time()
        # print((end_time-start_time)*1000)
        # event.wait(0.01)
        # rate.sleep()  # 控制循环频率

    yolov5_wrapper.destroy()
    cap.release()
    cv2.destroyAllWindows()


if __name__=="__main__":
    # 初始化ROS节点
    rospy.init_node('Car_on')
    main()
#!/usr/bin/env python3.8
# coding=utf-8
import os
import sys

path = os.path.abspath(".")
# # 核心
sys.path.insert(0,path + "/src/serial_pkg/scripts")
import rospy
import cv2
import numpy as np
from std_msgs.msg import UInt8, Int16
from geometry_msgs.msg import Twist
from main import send_data,limit_val


# 霍夫变换参数
rho = 1
theta = np.pi / 180
threshold = 15
min_line_len = 60  # 线的最小长度
max_line_gap = 100  # 线段之间最大允许间隙

#老学长的斜率
# slope_threshold_min = 0.2  # 斜率检测参数
# slope_threshold_max = 10

#符哥的斜率
slope_threshold_min = 0.5  # 斜率检测参数
slope_threshold_max = 5     #5
distance_threshold = 3  # 间距参数5
# exc_time = [-1.0, -1.0]


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
                        if ((x1 + x2) / 2) > 250 :
                            continue
                        else:
                            left_lines.append(line)
                    else:
                        if ((x1 + x2) / 2) < 180 or abs(k)<1.0:
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

# 移除左右偏差偏差过大的线子程序
# 迭代计算各条直线的斜率与斜率均值的差
# 逐一移除差值过大的线
# def clean_lines(lines, threshold):
#     slope = [(y2 - y1) / (x2 - x1) for line in lines for x1, y1, x2, y2 in line]
#     while len(lines) > 0:
#         mean = np.mean(slope)
#         diff = [abs(s - mean) for s in slope]
#         idx = np.argmax(diff)
#         if diff[idx] > threshold:
#             slope.pop(idx)
#             lines.pop(idx)
#         else:
#             break

def clean_lines(lines, threshold):
    slope = [(y2 - y1) / (x2 - x1) for line in lines for x1, y1, x2, y2 in line]
    if len(lines) > 0:
        mean = np.mean(slope)
        diff = [abs(s - mean) for s in slope]
        idx = -1
        for onediff in diff:
            idx += 1
            if onediff > threshold:
                slope.pop(idx)
                lines.pop(idx)
                idx = idx - 1
            else:
                pass



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

# def houghp_detect(input_roi_edges):
#     h = input_roi_edges.shape[0]
#     lines = hough_lines(input_roi_edges, rho, theta, threshold, min_line_len, max_line_gap)
#     left_results, right_results = [], []
#     if lines is not None:
#         left_lines, right_lines = choose_lines(lines, slope_threshold_min, slope_threshold_max)
#         for line in lines:
#             x1, y1, x2, y2 = line[0]
#             cv2.line(orgb, (x1, y1), (x2, y2), (255, 0, 0), 5)
#     else:
#         pass



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

    Last_turn_error = Turn_error
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
        # Turn_error = (left_results[0][0] + left_results[1][0]) / 2 + 130
        Turn_error = ((left_results[0][0] + 200-150)+(left_results[1][0] + 100-150))/2


    
    Turn_error = 0.8*Turn_error+0.2*Last_turn_error

    # Turn_error = ((left_results[0][0] + 200-150)+(left_results[1][0] + 100-150))/2
    # Turn_error = (left_results[0][0] + left_results[1][0]) / 2 + 140

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

def lane_detection(lane_img,show_image):
    leftx = []
    rightx = []
    left_results, right_results = [], []

    w_half = show_image.shape[1]//2 #w为图像中间，因为lane_img为裁剪一半的图像，原图像应该是640*320,所以真正的图像中间为320
    width_hp = 200#196
    
    lines = hough_lines(lane_img, rho, theta, threshold, min_line_len, max_line_gap)
    try:
        if len(lines) > 0:
            left_lines, right_lines = choose_lines(lines, slope_threshold_min, slope_threshold_max)
            L_num_point = 0
            L_point_x = 0
            L_point_y = 0
            L_point_xavg = 0
            L_point_yavg = 0
            R_num_point = 0
            R_point_x = 0
            R_point_y = 0
            R_point_xavg = 0
            R_point_yavg = 0
            if len(left_lines) > 0:
                clean_lines(left_lines, 0.1)
                for line in left_lines:
                    x1, y1, x2, y2 = line[0]
                    if abs(((x1 + x2 )// 2) - w_half) < 100:
                        cv2.line(show_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
                        leftx.append(x1)
                        leftx.append(x2)
                        L_point_x = L_point_x + x1 + x2
                        L_point_y = L_point_y + y1 + y2
                        L_num_point = L_num_point + 2
                    else:
                        continue

                L_point_xavg = L_point_x // L_num_point
                L_point_yavg = L_point_y // L_num_point


            else:
                pass

            if len(right_lines) > 0:
                clean_lines(right_lines, 0.1)
                for line in right_lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(show_image, (x1, y1), (x2, y2), (0, 255, 0), 5)
                    rightx.append(x1)
                    rightx.append(x2)
                    R_point_x = R_point_x + x1 + x2
                    R_point_y = R_point_y + y1 + y2
                    R_num_point = R_num_point + 2

                R_point_xavg = R_point_x // R_num_point
                R_point_yavg = R_point_y // R_num_point
            else:
                pass

            if L_point_xavg != 0 and R_point_xavg != 0:
                Mid_point_xavg = (L_point_xavg + R_point_xavg) // 2
            elif L_point_xavg != 0 and R_point_xavg == 0:
                Mid_point_xavg = L_point_xavg + (width_hp // 2)-1
            elif R_point_xavg != 0 and L_point_xavg == 0:
                Mid_point_xavg = R_point_xavg - (width_hp // 2)
            else:
                Mid_point_xavg = 150



            return L_point_xavg, L_point_yavg, R_point_xavg, R_point_yavg, Mid_point_xavg
    except:
        return 0,0,0,0,150


def get_error_hp(middle_point):
    #error_hp = 160 - middle_point
    error_hp = (150- middle_point)
    return error_hp

if __name__=="__main__":
    pass
# if __name__=="__main__":
#     # 初始化ROS节点
#     rospy.init_node('Lane_Detect')
#     yellow_low = np.array([26, 43, 83])
#     yellow_upp = np.array([90, 255, 255])

#     cap = cv2.VideoCapture(0)
#     # cap = cv2.VideoCapture("test.mp4")
#     width = 640  #定义摄像头获取图像宽度
#     height = 480   #定义摄像头获取图像长度
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)  #设置宽度
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  #设置长度
#     cap.set(cv2.CAP_PROP_FPS, 30)  #设置长度
#     twist = Twist()
#     twist.linear.x = 0.3
#     while True:
#         ret, img = cap.read()
#         if not ret:
#             break
#         # img = cv2.imread('img_wandao_2.JPG')

#         fps = cap.get(cv2.CAP_PROP_FPS)
#         col = 320	 # width
#         row = 240	 # height
#         row_p = 120 # prospect120
        
#         input_img = cv2.resize(img, (col, row), interpolation= cv2.INTER_AREA)
#         x, y = 0, 120
#         w, h = 320, 120
#         input_img = input_img[y:y+h, x:x+w]
#         input_img = cv2.resize(input_img, (300, 150), interpolation=cv2.INTER_LINEAR) 

#         input_img = cv2.GaussianBlur(input_img, (3, 3), 1, 1)   
#         hsv_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2HSV)
#         frame_process = cv2.inRange(hsv_img, yellow_low, yellow_upp)
#         kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
#         frame_process = cv2.dilate(frame_process,kernel,iterations=1)
#         kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)) #ksize=3,3
#         frame_process = cv2.erode(frame_process,kernel,iterations=1)
        


#         # hp_lines = hough_lines(frame_process, ) 
#         # 根据二值化的图像获取左右车道线
#         # left_results, right_results = get_lane_info(frame_process)
#     #     turn_error = get_turn_error_hp(line_image, left_results, right_results)
#         # cv2.line(input_img, (left_results[0][0], left_results[0][1]), (left_results[1][0], left_results[1][1]), (0, 0, 255), 2)
#         # cv2.line(input_img, (right_results[0][0], right_results[0][1]), (righ
#         # t_results[1][0], right_results[1][1]), (0, 0, 255), 2)

#         h = frame_process.shape[0]
#         w = frame_process.shape[1]
#         frame_process = frame_process[: ,0:(0+w//2)]
#         cv2.imshow('pre', frame_process)
#         L_point_xavg, L_point_yavg, R_point_xavg, R_point_yavg, Mid_point_xavg = lane_detection(frame_process)
#         turn = get_error_hp(Mid_point_xavg)

#         twist.angular.z = 0.016 * turn
#         limit_val(twist.angular.z, 0, -1.0)
#         send_data(twist.linear.x, twist.angular.z)



#         # Fit a second order polynomial to each using `np.polyfit`
#         # left_fit = np.polyfit(leftx, lefty, 2)
#         # right_fit = np.polyfit(rightx, righty, 2)
#         cv2.line(input_img, (L_point_xavg, L_point_yavg), (R_point_xavg, R_point_yavg), (0, 255, 255), 1)
#         cv2.putText(input_img, str(L_point_xavg), (150, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3, cv2.LINE_AA)
#         cv2.putText(input_img, str(R_point_xavg), (150, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3, cv2.LINE_AA)
#         cv2.putText(input_img, str(Mid_point_xavg), (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3,
#                     cv2.LINE_AA)
#         cv2.putText(input_img, str(fps), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 3,
#                         cv2.LINE_AA)
#         cv2.imshow('line',input_img)
#         if (cv2.waitKey(1) & 0xFF) == ord('q'):

#             break
#         # if cv2.waitKey(int(float(1/int(fps))*1000)) == 27:
#         #     break

#     send_data(0.0, 0.0)
#     cap.release()
#     cv2.destroyAllWindows()
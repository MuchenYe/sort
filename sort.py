#!/usr/bin/env python3.10
# coding=utf-8

import os
import sys
path = os.path.abspath(".")
sys.path.insert(0, path)
import cv2
import numpy as np
import math
import serial
import time
import threading
import apriltag  # Apriltag识别库
from datetime import datetime

# 导入自定义模块（需根据实际功能实现）
import lineSortMap2

# 初始化摄像头
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("无法打开摄像头")
    exit()

# 初始化串口
try:
    uart = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
    print("串口初始化成功")
except Exception as e:
    print(f"串口初始化失败: {e}")
    uart = None

'''
    3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
    cx: 偏右减小, 偏左增加
    cy: 偏前减小，偏后增加
    cz: 偏高减小，偏低增加
'''
# cx = 0
# cy = 0
# cz = 0

# 运行状态
run_app_status = 0

def send_command(command):
    """发送串口命令"""
    if uart:
        try:
            uart.write(command.encode())
            print(f"发送命令: {command}")
        except Exception as e:
            print(f"串口发送失败: {e}")
    else:
        print(f"模拟发送命令: {command}")

class RobotMoveCmd:
    def car_move(self, speed1, speed2, speed3, speed4):
        """控制小车运动"""
        send_command(f"$Car:{float(speed1)},{float(speed2)},{float(speed3)},{float(speed4)}!")

class LineSortMap2:
    # 设置阈值，如果是黑线，GRAYSCALE_THRESHOLD = [(0, 64)]；
    # 如果是白线，GRAYSCALE_THRESHOLD = [(128，255)]
    GRAYSCALE_THRESHOLD = [(0, 64)]
    
    # ROIs格式: (x, y, w, h, weight, roi_id)
    # 每个roi为(x, y, w, h)，线检测算法将尝试找到每个roi中最大的blob的质心。
    # 然后用不同的权重对质心的x位置求平均值，其中最大的权重分配给靠近图像底部的roi，
    # 较小的权重分配给下一个roi，以此类推。
    # roi代表三个取样区域，（x,y,w,h,weight）,代表左上顶点（x,y）宽高分别为w和h的矩形，
    # weight为当前矩形的权值。注意本例程采用的QQVGA图像大小为160x120，roi即把图像横分成三个矩形。
    # 三个矩形的阈值要根据实际情况进行调整，离机器人视野最近的矩形权值要最大，
    # 如上图的最下方的矩形，即(0, 100, 200, 20, 0.7,1)（最后一个值1是用来记录的）
    ROIS = [
        (0, 100, 200, 20, 0.5, 1),
        (0, 75, 200, 20, 0.2, 2),
        (0, 50, 200, 20, 0.2, 3),
        (0, 25, 200, 20, 0.05, 4),
        (0, 0, 200, 20, 0.05, 5)
    ]
    
    # 颜色阈值(HSV格式)
    red_threshold = (0, 100, 20, 100, 20, 127)
    blue_threshold = (90, 50, 50, 120, 255, 255)
    green_threshold = (35, 43, 46, 77, 255, 255)
    yellow_threshold = (26, 43, 46, 35, 255, 255)
    track_color_threshold = blue_threshold  # 追踪的颜色
         
    def init(self):
        """初始化函数"""
        # 机械臂移动位置
        self.move_x = 0
        self.move_y = 160

        self.adjust_position = 0  # 用来判断小车状态
        self.cap_block_cnt = 0  # 用来计数抓取物块数量
        self.crossing_flag = 0  # 标记路口情况计数，判断是否经过一个路口
        self.is_line_flag = 1  # 是否可以巡线标志
        self.cap_color_status = 0  # 抓取物块颜色标志，用来判断物块抓取
        self.move_status = 0  # 机械臂移动的方式
        self.crossing_record_cnt = 0  # 用来记录经过的路口数量
        self.mid_adjust_position = 0  # 小车到中间横线时需要调整身位后在寻找分拣区，变量为标志位
        self.mid_block_cnt = 0  # 用来记录机械臂已对准物块计数，防止误差
        self.over_flag = 0  # 用来标记小车180度翻转
        self.mid_over_flag = 0  # 记录小车翻转到一半
        self.mid_over_cnt = 0  # 记录小车翻转到一半计数
        self.adjust_position_cnt = 0  # 调整身位计数
        self.car_back_flag = 0  # 小车后退还是前进标志

        self.crossing_cnt = 0  # MAP3路口计数
        self.speed_motor1 = 0
        self.speed_motor2 = 0
        self.speed_motor3 = 0
        self.speed_motor4 = 0

        # 物块中心点
        self.mid_block_cx = 80  # 图像宽度一半
        self.mid_block_cy = 60  # 图像高度一半
        
        # 机械臂偏移量
        self.cx = 0
        self.cy = 0
        self.cz = 0
    
    def line_walk(self, frame):
        """巡线功能"""
        weight_sum = 0
        centroid_sum = 0
        
        # 基础速度
        base_speed = 0.1
        
        # 记录各ROI中的线块
        blob_roi1 = 0
        blob_roi2 = 0
        blob_roi3 = 0
        blob_roi4 = 0
        blob_roi5 = 0
        
        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 寻找各ROI中的线
        for r in self.ROIS:
            x, y, w, h, weight, roi_id = r
            roi = (x, y, w, h)
            
            # 二值化ROI区域
            roi_gray = gray[y:y+h, x:x+w]
            _, binary_roi = cv2.threshold(roi_gray, 64, 255, cv2.THRESH_BINARY_INV)
            
            # 寻找轮廓
            contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # 找到最大轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 50:  # 过滤小轮廓
                    # 计算轮廓质心
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"]) + x
                        cy = int(M["m01"] / M["m00"]) + y
                        
                        # 绘制轮廓和质心
                        cv2.drawContours(frame, [largest_contour], -1, (0, 255, 0), 2)
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"ROI{roi_id}", (x, y-10), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        
                        # 记录各ROI中的线块
                        if roi_id == 1:
                            blob_roi1 = (x, y, w, h)
                        elif roi_id == 2:
                            blob_roi2 = (x, y, w, h)
                        elif roi_id == 3:
                            blob_roi3 = (x, y, w, h)
                        elif roi_id == 4:
                            blob_roi4 = (x, y, w, h)
                        elif roi_id == 5:
                            blob_roi5 = (x, y, w, h)
                        
                        centroid_sum += cx * weight
                        weight_sum += weight
        
        # 原地翻转处理
        if self.over_flag == 1:
            if blob_roi3 == 0 and blob_roi5 == 0 and self.mid_over_flag == 0:   # 摄像头已经识别不到线，说明翻转到一半
                self.mid_over_cnt += 1
                if self.mid_over_cnt > 5:
                    self.mid_over_flag = 1
                    self.mid_over_cnt = 0
            elif blob_roi1 != 0 and blob_roi3 != 0 and blob_roi5 != 0 and self.mid_over_flag == 1:  # 重新识别到三条范围内的线，取消翻转，重新开始巡线
                self.over_flag = 0
                self.mid_over_flag = 0
                time.sleep(0.2)
            robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)
            return
        
        # **********************************判断路口情况，检测两个自定义的范围都检测到路口就说明经过一个路口***********************************************
        if self.crossing_flag == 0 and ((blob_roi3 != 0 and blob_roi3[2] > 90) or (blob_roi4 != 0 and blob_roi4[2] > 90)):
            self.crossing_flag = 1
        elif self.crossing_flag == 1 and blob_roi1 != 0 and blob_roi1[2] > 90:  # 1号ROIS检测到路口
            self.crossing_flag = 2
        
        # 巡线控制
        if weight_sum > 0 and self.is_line_flag == 1:   # 如果识别到线条或者没有特殊情况，开始巡线
            center_pos = centroid_sum / weight_sum  # 计算线的中心位置

            # 将center_pos转换为一个偏角。我们用的是非线性运算，所以越偏离直线，响应越强。
            # 非线性操作很适合用于这样的算法的输出，以引起响应“触发器”。
            # 80是X的一半，60是Y的一半。
            # 下面的等式只是计算三角形的角度，其中三角形的另一边是中心位置与中心的偏差，相邻边是Y的一半。
            # 这样会将角度输出限制在-45至45度左右。（不完全是-45至45度）。
            # 角度计算.80 60 分别为图像宽和高的一半，图像大小为160x120.
            # 注意计算得到的是弧度值
            # 将计算结果的弧度值转化为角度值
            # 现在你有一个角度来告诉你该如何转动机器人。
            # 通过该角度可以合并最靠近机器人的部分直线和远离机器人的部分直线，以实现更好的预测。
            # 计算偏转角度
            deflection_angle = -math.atan((center_pos - self.mid_block_cx) / self.mid_block_cy)
            deflection_angle = math.degrees(deflection_angle)
            
            # 调整身位处理
            if self.mid_adjust_position == 1:
                if blob_roi5 != 0 and blob_roi5[2] > 100 and abs(deflection_angle) < 5: #身位调整完毕
                    self.adjust_position_cnt += 1
                    robot_move_cmd.car_move(0, 0, 0, 0)
                    if self.adjust_position_cnt > 10:   #调整身位成功
                        self.adjust_position_cnt = 0
                        self.is_line_flag = 0
                        self.car_back_flag = 0
                        self.mid_adjust_position = 0
                        self.move_x = 120   #旋转机械臂寻找颜色框
                        if self.crossing_record_cnt == 3:   #第三个路口的机械臂旋转的方向不同
                            self.move_x = -120
                        self.move_y = 50
                        kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                        time.sleep(1.2)
                    return
                # 小车身位太靠前，需后退
                elif blob_roi5 == 0 or (blob_roi4 != 0 and blob_roi4[2] > 100) or \
                     (blob_roi3 != 0 and blob_roi3[2] > 100) or (blob_roi2 != 0 and blob_roi2[2] > 100) or \
                     (blob_roi1 != 0 and blob_roi1[2] > 100):
                    self.car_back_flag = 1
                # 小车倒退出线
                elif blob_roi5 != 0 and blob_roi5[2] < 50:
                    self.car_back_flag = 0
                self.adjust_position_cnt = 0
            
            # 计算电机速度
            if self.mid_adjust_position == 1 or self.car_back_flag == 1:
                if self.car_back_flag == 1 and self.crossing_record_cnt != 12:  #小车需倒退
                    if deflection_angle < -6:   #右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 6:  #左转
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                elif self.car_back_flag == 0 and self.crossing_record_cnt != 12:
                    # 计算每个电机的速度，根据偏转角度进行调整
                    if deflection_angle < -10:  #右转
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = -base_speed / 3
                    elif deflection_angle > 10: #左转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = base_speed / 3
                    else:
                        self.speed_motor1 = base_speed / 3
                        self.speed_motor2 = base_speed / 3
                        self.speed_motor3 = base_speed / 3
                        self.speed_motor4 = base_speed / 3
                elif self.car_back_flag == 1 and self.crossing_record_cnt == 12:
                    if deflection_angle < -20:  #右转
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 2
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 2
                    elif deflection_angle > 20: #左转
                        self.speed_motor1 = -base_speed / 2
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 2
                        self.speed_motor4 = -base_speed / 3
                    else:
                        self.speed_motor1 = -base_speed / 3
                        self.speed_motor2 = -base_speed / 3
                        self.speed_motor3 = -base_speed / 3
                        self.speed_motor4 = -base_speed / 3
            else:
                if deflection_angle < 0:    #右转
                # 右侧电机速度增加，左侧电机速度减少
                    self.speed_motor1 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 - abs(deflection_angle) / 45)
                else:   #左转
                # 左侧电机速度增加，右侧电机速度减少
                    self.speed_motor1 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor2 = base_speed * (1 + abs(deflection_angle) / 45)
                    self.speed_motor3 = base_speed * (1 - abs(deflection_angle) / 45)
                    self.speed_motor4 = base_speed * (1 + abs(deflection_angle) / 45)
            
            robot_move_cmd.car_move(self.speed_motor1, self.speed_motor2, self.speed_motor3, self.speed_motor4)
        else:
            robot_move_cmd.car_move(0, 0, 0, 0)
            time.sleep(0.5)
    
    def run(self, frame, cx=0, cy=0, cz=0):
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏右减小, 偏左增加
            cy: 偏前减小，偏后增加
            cz: 偏高减小，偏低增加
        '''
        self.cx = cx
        self.cy = cy
        self.cz = cz
        
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        color_read_succeed = 0  #是否识别到颜色
        color_status = 0
        
        if self.is_line_flag == 1:
            # 巡线模式
            self.line_walk(frame)
        else:
            # 颜色识别模式
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 红色阈值范围(HSV)
            red_lower1 = np.array([0, 100, 20])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([160, 100, 20])
            red_upper2 = np.array([180, 255, 255])
            mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
            mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
            mask_red = mask_red1 + mask_red2
            
            # 蓝色阈值
            mask_blue = cv2.inRange(hsv, 
                                  np.array(self.blue_threshold[:3]), 
                                  np.array(self.blue_threshold[3:]))
            
            # 绿色阈值
            mask_green = cv2.inRange(hsv, 
                                    np.array(self.green_threshold[:3]), 
                                    np.array(self.green_threshold[3:]))
            
            # 寻找最大色块
            red_blobs = self.find_largest_blob(mask_red)
            blue_blobs = self.find_largest_blob(mask_blue)
            green_blobs = self.find_largest_blob(mask_green)
            
            if red_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'R'):
                color_status = 'R'
                color_read_succeed = 1
                block_cx, block_cy = red_blobs
                cv2.circle(frame, (block_cx, block_cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, "red", (block_cx-10, block_cy-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            elif blue_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'B'):
                color_status = 'B'
                color_read_succeed = 1
                block_cx, block_cy = blue_blobs
                cv2.circle(frame, (block_cx, block_cy), 5, (255, 0, 0), -1)
                cv2.putText(frame, "blue", (block_cx-10, block_cy-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            elif green_blobs and (self.cap_color_status == 0 or self.cap_color_status == 'G'):
                color_status = 'G'
                color_read_succeed = 1
                block_cx, block_cy = green_blobs
                cv2.circle(frame, (block_cx, block_cy), 5, (0, 255, 0), -1)
                cv2.putText(frame, "green", (block_cx-10, block_cy-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 判断路口情况（底盘运动控制）
        if self.crossing_flag == 2: #找到路口
            self.crossing_flag = 0  
            self.crossing_record_cnt += 1   #记录经过的路口数量
            if self.crossing_record_cnt == 2 or self.crossing_record_cnt == 5 or self.crossing_record_cnt == 9: #经过的路口数量在物品区，小车停止，开始颜色识别
                self.is_line_flag = 0
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)    #前进
                time.sleep(0.5)
                robot_move_cmd.car_move(0, 0, 0, 0)
            elif self.crossing_record_cnt == 3:  #第3个路口小车需要右转,改为颜色识别，旋转机械臂到左边
                self.mid_adjust_position = 1    #调整身位寻找分拣区
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep(2.5)
                robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  #右转
                time.sleep(1.5)
            elif self.crossing_record_cnt == 4 or self.crossing_record_cnt == 11:   #第4,11个路口小车需要右转
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep(2.5)
                robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  #右转
                time.sleep(1.5)
            elif self.crossing_record_cnt == 6: #第6个路口小车需要左转，改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep(2.5)
                robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)  #左转
                time.sleep(1.5)
            elif self.crossing_record_cnt == 7: #第7个路口小车需要旋转180度
                robot_move_cmd.car_move(-0.1, 0.1, -0.1, 0.1)
                time.sleep(1.0)
                self.over_flag = 1  #原地翻转
                return
            elif self.crossing_record_cnt == 8: #第8个路口小车需要左转
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep(2.5)
                robot_move_cmd.car_move(-0.2, 0.2, -0.2, 0.2)  #左转
                time.sleep(1.5)
            elif self.crossing_record_cnt == 10:    #第10个路口小车需要右转，改为颜色识别，旋转机械臂到右边
                self.mid_adjust_position = 1
                robot_move_cmd.car_move(0.1, 0.1, 0.1, 0.1)
                time.sleep(2.5)
                robot_move_cmd.car_move(0.2, -0.2, 0.2, -0.2)  #右转
                time.sleep(1.5)
            elif self.crossing_record_cnt == 12:    #第12个路口小车回到原点，需要原地旋转180度后倒车
                robot_move_cmd.car_move(0, 0, 0, 0)
                kinematic.kinematics_move(self.move_x, self.move_y, 70, 500)
                time.sleep(0.6)
                self.crossing_flag = 1  #标记路口
                self.car_back_flag = 1  #前进后退
                self.over_flag = 1  #原地翻转
                return
            elif self.crossing_record_cnt == 13:    #第13个路口小车回到原点
                robot_move_cmd.car_move(0, 0, 0, 0)
                self.is_line_flag = 0   #取消巡线
                return
        
        # 机械臂控制逻辑
        if color_read_succeed == 1: #识别到颜色或者到路口
            if self.move_status == 0:   #第0阶段：机械臂寻找物块位置
                if abs(block_cx - self.mid_block_cx) > 3:
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.5
                    else:
                        self.move_x -= 0.5
                if abs(block_cy - self.mid_block_cy) > 3 and self.move_y > 1:
                    if block_cy > self.mid_block_cy:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:   #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10: #计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 1
                        self.cap_color_status = color_status    #色块颜色
                else:
                    self.mid_block_cnt = 0
                    kinematic.kinematics_move(self.move_x, self.move_y, 50, 0)
                time.sleep(0.01)
            
            elif self.move_status == 1: #第1阶段：机械臂抓取物块
                self.move_status = 2
                l = math.sqrt(self.move_x **2 + self.move_y** 2)
                sin_val = self.move_y / l
                cos_val = self.move_x / l
                self.move_x = (l + 79 + cy) * cos_val + cx
                self.move_y = (l + 79 + cy) * sin_val
                time.sleep(0.1)
                self.send_command("{#005P1000T1000!}")
                time.sleep(0.1)
                kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  #移动机械臂到物块上方
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x, self.move_y, -75 + cz, 1000)    #移动机械臂下移到物块
                time.sleep(1.2)
                self.send_command("{#005P1700T1000!}")    #机械臂抓取物块
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)  #移动机械臂抬起
                time.sleep(1.2)
                self.move_x = 0 #机械臂归位
                self.move_y = 160
                kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep(1.2)
                self.crossing_flag = 0  #重置路口
                self.mid_block_cnt = 0  #重置计数
                self.is_line_flag = 1   #巡线标志
                self.over_flag = 1  #翻转标志
            
            elif self.move_status == 2: #第2阶段：调整身位
                if block_cx - self.mid_block_cx > 20:
                    if self.crossing_record_cnt == 3:   #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            robot_move_cmd.car_move(0.05, 0.05, 0.05, 0.05)    #前进
                    else:
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            robot_move_cmd.car_move(-0.05, -0.05, -0.05, -0.05)    #后退
                    return
                elif block_cx - self.mid_block_cx < -20:
                    if self.crossing_record_cnt == 3:   #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                        if self.adjust_position != 4:
                            self.adjust_position = 4
                            robot_move_cmd.car_move(-0.05, -0.05, -0.05, -0.05)    #后退
                    else:
                        if self.adjust_position != 1:
                            self.adjust_position = 1
                            robot_move_cmd.car_move(0.05, 0.05, 0.05, 0.05)    #前进
                    return
                else:   #调整完毕，停止
                    if self.adjust_position != 5:
                        self.adjust_position = 5
                        robot_move_cmd.car_move(0, 0, 0, 0)
                        self.move_status = 3
            
            elif self.move_status == 3: #第3阶段：机械臂寻找放下物块的框框
                if abs(block_cx - self.mid_block_cx) > 5:
                    if block_cx > self.mid_block_cx and self.move_y > 1:
                        if self.crossing_record_cnt == 3:   #路口处于3的时候颜色区在机械臂左侧，其他时候在右侧
                            self.move_y += 0.5
                        else:
                            self.move_y -= 0.5
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_y -= 0.5
                        else:
                            self.move_y += 0.5
                if abs(block_cy - self.mid_block_cy) > 5:
                    if block_cy > self.mid_block_cy:
                        if self.crossing_record_cnt == 3:
                            self.move_x += 0.3
                        else:
                            self.move_x -= 0.3
                    else:
                        if self.crossing_record_cnt == 3:
                            self.move_x -= 0.3
                        else:
                            self.move_x += 0.3
                if abs(block_cy - self.mid_block_cy) <= 5 and abs(block_cx - self.mid_block_cx) <= 5:   #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 10: #计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 4
                else:
                    self.mid_block_cnt = 0
                    kinematic.kinematics_move(self.move_x, self.move_y, 70, 0)
                time.sleep(0.01)
            
            elif self.move_status == 4:#第4阶段：机械臂放下物块并归位
                self.move_status = 0
                l = math.sqrt(self.move_x ** 2 + self.move_y ** 2)
                sin_val = self.move_y / l
                cos_val = self.move_x / l
                self.move_x = (l + 86 + cy) * cos_val + cx
                self.move_y = (l + 86 + cy) * sin_val
                kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x, self.move_y, -50 + cz, 1000)
                time.sleep(1.2)
                self.send_command("{#005P1000T1000!}")
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)
                self.move_x = 0 #机械臂归位
                self.move_y = 160
                kinematic.kinematics_move(self.move_x, self.move_y, 50, 1000)
                time.sleep(1.2)
                self.mid_block_cnt = 0
                self.is_line_flag = 1   #巡线标志
                self.cap_color_status = 0   #抓取物块颜色标志
                self.adjust_position = 0    #小车状态
    
    def find_largest_blob(self, mask):
        """寻找最大色块并返回质心"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            if area > 25:   #过滤小面积色块
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)
        return None

class Kinematics:
    # 设置四个关节的长度 mm
    # 放大10倍
    L0 = 1000
    L1 = 1050
    L2 = 880
    L3 = 1550

    pi = math.pi  # 使用Python标准库的pi值
    time = 1500  # 运动默认时间(ms)

    def kinematics_analysis(self, x: float, y: float, z: float, alpha: float) -> str:
        """
        机械臂逆运动学分析
        x,y 为映射到平面的坐标(mm)
        z为距离地面的距离(mm)
        alpha 为爪子和平面的夹角(度) -25~-65范围比较好
        """
        # 放大10倍
        x = x * 10
        y = y * 10
        z = z * 10

        l0 = float(self.L0)
        l1 = float(self.L1)
        l2 = float(self.L2)
        l3 = float(self.L3)

        # 计算基座旋转角度
        if x == 0:
            theta6 = 0.0
        else:
            theta6 = math.atan(x/y) * 270.0 / self.pi

        # 坐标转换
        y = math.sqrt(x*x + y*y)
        y = y - l3 * math.cos(alpha * self.pi / 180.0)
        z = z - l0 - l3 * math.sin(alpha * self.pi / 180.0)
        
        # 检查位置有效性
        if z < -l0:
            return 1
        if math.sqrt(y*y + z*z) > (l1 + l2):
            return 2

        # 计算关节角度
        try:
            ccc = math.acos(y / math.sqrt(y * y + z * z))
            bbb = (y*y + z*z + l1*l1 - l2*l2) / (2*l1*math.sqrt(y*y + z*z))
            
            if bbb > 1 or bbb < -1:
                return 5
            
            if z < 0:
                zf_flag = -1
            else:
                zf_flag = 1

            theta5 = ccc * zf_flag + math.acos(bbb)
            theta5 = theta5 * 180.0 / self.pi
            
            if theta5 > 180.0 or theta5 < 0.0:
                return 6

            aaa = -(y*y + z*z - l1*l1 - l2*l2) / (2*l1*l2)
            if aaa > 1 or aaa < -1:
                return 3

            theta4 = math.acos(aaa)
            theta4 = 180.0 - theta4 * 180.0 / self.pi
            
            if theta4 > 135.0 or theta4 < -135.0:
                return 4

            theta3 = alpha - theta5 + theta4
            if theta3 > 90.0 or theta3 < -90.0:
                return 7

            # 转换为舵机角度
            servo_angle0 = theta6
            servo_angle1 = theta5 - 90
            servo_angle2 = theta4
            servo_angle3 = theta3

            # 转换为PWM值 (范围: 500-2500)
            servo_pwm0 = int(1500 - 2000.0 * servo_angle0 / 270.0)
            servo_pwm1 = int(1500 + 2000.0 * servo_angle1 / 270.0)
            servo_pwm2 = int(1500 + 2000.0 * servo_angle2 / 270.0)
            servo_pwm3 = int(1500 - 2000.0 * servo_angle3 / 270.0)

            # 机械臂第三个舵机反位
            servo_pwm3 = 3000 - servo_pwm3

            # 生成舵机控制指令
            arm_str = (f"{{#000P{servo_pwm0:0>4d}T{self.time:0>4d}!"
                      f"#001P{servo_pwm1:0>4d}T{self.time:0>4d}!"
                      f"#002P{servo_pwm2:0>4d}T{self.time:0>4d}!"
                      f"#003P{servo_pwm3:0>4d}T{self.time:0>4d}!}}")

            return arm_str
        except Exception as e:
            return f"计算异常: {str(e)}"

    # 机械臂逆运动控制
    def kinematics_move(self, x: float, y: float, z: float, time1: int) -> int:
        """
        控制机械臂移动到指定位置
        x, y, z: 目标位置坐标(mm)
        time1: 运动时间(ms)
        """
        if y < 0:
            print("错误: Y坐标不能为负")
            return 0
            
        self.time = time1

        # 寻找最佳角度
        best_angle = 0
        found = False

        # 在-135度到0度之间寻找合适的角度
        for i in range(0, -136, -1):
            result = self.kinematics_analysis(x, y, z, i)
            if isinstance(result, str) and not result.startswith("错误"):
                best_angle = i
                found = True

        # 使用找到的最佳角度控制机械臂
        if found:
            arm_str = self.kinematics_analysis(x, y, z, best_angle)
            if isinstance(arm_str, str) and not arm_str.startswith("错误"):
                send_command(arm_str)
                return 1
        
        return 0
class ColorPalletizer:
    red_threshold = (160, 180, 50, 255, 50, 255)
    blue_threshold = (100, 125, 120, 255, 100, 255)
    green_threshold = (30, 90, 40, 255, 30, 255)
    yellow_threshold = (0, 15, 50, 255, 50, 255)

    def init(self):
        """初始化系统"""
        
        # 物块中心点（图像坐标系）
        self.mid_block_cx = 150
        self.mid_block_cy = 150
        
        self.move_status = 0  # 运行状态标记
        self.mid_block_cnt = 0  # 机械臂移到色块中心计数
        self.palletizer_cnt = 0  # 记录码垛的数量
        
        # 机械臂初始位置
        self.move_x = 0
        self.move_y = 100
        kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
        time.sleep(1)
        
    def find_blobs(self, img, threshold, x_stride=15, y_stride=15, pixels_threshold=25):
        """在图像中查找色块"""
        # 将OpenMV的阈值转换为OpenCV的HSV阈值
        h_min, h_max, s_min, s_max, v_min, v_max = threshold
        
        # 转换图像到HSV色彩空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 创建掩码
        mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
        
        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        blobs = []
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < pixels_threshold or area > 400:
                continue
                
            # 计算边界框和中心点
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            blobs.append((x, y, w, h, 0, cx, cy))
            
        return blobs
        
    def run(self, frame, cx=0, cy=0, cz=0):
        """运行颜色分拣功能"""
        # 物块中心点
        block_cx = self.mid_block_cx
        block_cy = self.mid_block_cy
        
        color_read_succed = 0  # 是否识别到颜色
        
        # 查找色块
        red_blobs = self.find_blobs(frame, self.red_threshold)
        blue_blobs = self.find_blobs(frame, self.blue_threshold)
        green_blobs = self.find_blobs(frame, self.green_threshold)
        
        # 首先进行色块检测
        if red_blobs:  # 红色
            color_read_succed = 1
            # 选择最大的色块
            largest_blob = max(red_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            # 绘制检测结果
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "red", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            block_cx = c_x
            block_cy = c_y
            
        elif blue_blobs:  # 蓝色
            color_read_succed = 1
            largest_blob = max(blue_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "blue", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            block_cx = c_x
            block_cy = c_y
            
        elif green_blobs:  # 绿色
            color_read_succed = 1
            largest_blob = max(green_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "green", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            block_cx = c_x
            block_cy = c_y
        print("move_status:", self.move_status)
        print("color_read_succed:", color_read_succed)
        # print("cx:", cx)
        # print("cy:", cy)
        # print("x:", x)
        # print("y:", y)
        # print("w:", w)
        # print("h:", h)
        # 运动机械臂
        if color_read_succed == 1 or (self.move_status == 1):  # 识别到颜色
            if self.move_status == 0:  # 第0阶段：机械臂寻找物块位置
                if abs(block_cx - self.mid_block_cx) > 3:
                    if block_cx > self.mid_block_cx:
                        self.move_x += 0.2
                    else:
                        self.move_x -= 0.2
                if abs(block_cy - self.mid_block_cy) > 3:
                    if block_cy > self.mid_block_cy and self.move_y > 80:
                        self.move_y -= 0.3
                    else:
                        self.move_y += 0.3
                
                # 寻找到物块，机械臂进入第二阶段
                if abs(block_cy - self.mid_block_cy) <= 3 and abs(block_cx - self.mid_block_cx) <= 3:
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:  # 计数10次对准物块，防止误差
                        self.mid_block_cnt = 0
                        self.move_status = 1
                    print("move_x", self.move_x)
                    print("move_y", self.move_y)
                else:
                    self.mid_block_cnt = 0
                    print("move_x", self.move_x)
                    print("move_y", self.move_y)
                    kinematic.kinematics_move(self.move_x, self.move_y, 70, 10)
                
                time.sleep(0.01)  # 10ms
                
            elif self.move_status == 1:  # 第1阶段：机械臂抓取物块
                l = math.sqrt(self.move_x * self.move_x + self.move_y * self.move_y)
                sin_val = self.move_y / l
                cos_val = self.move_x / l
                self.move_x = (l + 85 + cy) * cos_val + cx
                self.move_y = (l + 85 + cy) * sin_val

                print("move_x", self.move_x)
                print("move_y", self.move_y)

                time.sleep(0.1)  # 100ms
                kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)  # 移动机械臂到物块上方
                time.sleep(0.1)  # 100ms
                send_command("{#005P1000T1000!}")  # 张开爪子
                time.sleep(1)  # 1200ms

                kinematic.kinematics_move(self.move_x, self.move_y, 25 + cz, 1000)  # 移动机械臂下移到物块
                print("11111111111111")
                time.sleep(2)  # 1200ms
                
                send_command("{#005P1700T1000!}")  # 机械爪抓取物块
                time.sleep(1.2)  # 1200ms
                
                kinematic.kinematics_move(self.move_x, self.move_y, 120, 1000)  # 移动机械臂抬起
                print("2222222222")
                time.sleep(1.2)  # 1200ms
                
                # 机械臂旋转到要方向物块的指定位置
                self.move_x = 200
                self.move_y = 100
                kinematic.kinematics_move(self.move_x, self.move_y, 120, 1000)
                time.sleep(1.2)  # 1200ms
                
                # 码垛位置控制
                if self.palletizer_cnt == 0:  # 第1次码垛
                    kinematic.kinematics_move(self.move_x, self.move_y, 25 + cz, 1000)
                elif self.palletizer_cnt == 1:  # 第2次码垛
                    kinematic.kinematics_move(self.move_x - 5, self.move_y, 25 + cz + 50, 1000)
                elif self.palletizer_cnt == 2:  # 第3次码垛
                    kinematic.kinematics_move(self.move_x, self.move_y - 5, 25 + cz + 40 + 50, 1000)
                time.sleep(1.2)  # 1200ms
                
                send_command("{#005P1000T1000!}")  # 张开爪子
                time.sleep(2)  # 1200ms
                
                kinematic.kinematics_move(self.move_x, self.move_y, 120, 1000)  # 移动机械臂抬起
                time.sleep(1.2)  # 1200ms
                
                # 机械臂归位
                self.move_x = 0
                self.move_y = 100
                kinematic.kinematics_move(self.move_x, self.move_y, 70, 1000)
                time.sleep(1.2)  # 1200ms
                
                self.move_status = 0
                self.palletizer_cnt = self.palletizer_cnt + 1

def handle_uart_command():
    """处理串口命令（单独线程）"""
    global run_app_status
    while True:
        if uart and uart.in_waiting > 0:
            try:
                string = uart.read(uart.in_waiting).decode('utf-8')
                print(f"接收串口数据: {string}")
                
                # if "#RunStop!" in string:  # 停止所有运行并复位
                #     run_app_status = 0
                #     for j in range(3):
                #         send_command("$MV0!")
                #         time.sleep(0.1)
                #     send_command("$Car:0,0,0,0!")
                #     send_command("$KMS:0,160,50,1000!")
                #     led_control(0)
                # elif "#CarColorTrace!" in string:  # 小车颜色追踪
                #     run_app_status = 1
                #     for j in range(3):
                #         send_command("$MV1!")
                #         time.sleep(0.1)
                #     car_Color_Trace.init()
                # elif "#Map3LineWalk!" in string:  # 巡线地图3
                #     run_app_status = 2
                #     for j in range(3):
                #         send_command("$MV2!")
                #         time.sleep(0.1)
                #     line_Map3.init()
                # elif "#ArmColorTrace!" in string:  # 机械臂颜色追踪
                #     run_app_status = 3
                #     for j in range(3):
                #         send_command("$MV3!")
                #         time.sleep(0.1)
                #     arm_Color_Trace.init()
                # elif "#AprilTagSort!" in string:  # 二维码标签分拣
                #     run_app_status = 4
                #     for j in range(3):
                #         send_command("$MV4!")
                #         time.sleep(0.1)
                #     apriltag_Sort.init()
                # elif "#AprilTagStack!" in string:  # 二维码标签码垛
                #     run_app_status = 5
                #     for j in range(3):
                #         send_command("$MV5!")
                #         time.sleep(0.1)
                #     apriltag_Palletizer.init()
                # elif "#TraceGrasp!" in string:  # 颜色追踪抓取
                #     run_app_status = 6
                #     for j in range(3):
                #         send_command("$MV6!")
                #         time.sleep(0.1)
                #     car_Color_Trace_Grasp.init()
                # elif "#Map1LineWalk!" in string:  # 地图1巡线
                #     run_app_status = 7
                #     for j in range(3):
                #         send_command("$MV7!")
                #         time.sleep(0.1)
                #     line_Sort_Map1.init()
                # elif "#Map2LineWalk!" in string:  # 地图2巡线
                #     run_app_status = 8
                #     for j in range(3):
                #         send_command("$MV8!")
                #         time.sleep(0.1)
                #     line_Sort_Map2.init()
                # elif "#FaceTrack!" in string:  # 人脸识别
                #     run_app_status = 9)
                if "#Map2LineWalk!" in string:  # 地图2巡线
                    run_app_status = 8
                    for j in range(3):
                        send_command("$MV8!")
                        time.sleep(0.1)
                    line_Sort_Map2.init()
            except Exception as e:
                print(f"处理串口数据异常: {e}")
        time.sleep(0.01)  # 避免CPU占用过高

class ColorSort():
    red_threshold = (0, 10, 150, 255, 100, 255)
    blue_threshold = (100, 130, 150, 255, 80, 255)
    green_threshold = (45, 85, 120, 255, 50, 255)
    yellow_threshold = (20, 35, 100, 255, 100, 255)

    cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
    #机械臂移动位置
    move_x=0
    move_y=100

    #摄像头中点
    mid_block_cx=150
    mid_block_cy=150

    mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差
    move_status=0#机械臂移动的方式

    def init(self):#初始化

        self.cap_color_status=0#抓取物块颜色标志，用来判断物块抓取
        #机械臂移动位置
        self.move_x=0
        self.move_y=100

        self.mid_block_cnt=0#用来记录机械臂已对准物块计数，防止误差
        self.move_status=0#机械臂移动的方式

        kinematic.kinematics_move(self.move_x,self.move_y,70,1000)
        time.sleep(1)
    def find_blobs(self, img, threshold, x_stride=15, y_stride=15, pixels_threshold=25):
        """在图像中查找色块"""
        # 将OpenMV的阈值转换为OpenCV的HSV阈值
        h_min, h_max, s_min, s_max, v_min, v_max = threshold
        
        # 转换图像到HSV色彩空间
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # 创建掩码
        mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))
        
        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        blobs = []
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            if area < pixels_threshold or area > 400:
                continue
                
            # 计算边界框和中心点
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            blobs.append((x, y, w, h, 0, cx, cy))  # 模拟OpenMV的blobs格式
            
        return blobs
        
    def run(self, frame, cx=0,cy=0,cz=0):#运行功能
        '''
            3个变量控制机械臂抓取色块时的偏移量,如果机械臂抓取色块失败则调整变量
            cx: 偏右减小, 偏左增加
            cy: 偏前减小，偏后增加
            cz: 偏高减小，偏低增加
        '''
        #物块中心点
        block_cx=self.mid_block_cx
        block_cy=self.mid_block_cy
        color_read_succed=0#是否识别到颜色
        color_status=0

        # 查找色块
        red_blobs = self.find_blobs(frame, self.red_threshold)
        blue_blobs = self.find_blobs(frame, self.blue_threshold)
        green_blobs = self.find_blobs(frame, self.green_threshold)
        
        # 首先进行色块检测
        if red_blobs:  # 红色
            color_read_succed = 1
            # 选择最大的色块
            largest_blob = max(red_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            # 绘制检测结果
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (255, 0, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "red", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            
            block_cx = c_x
            block_cy = c_y
            
        elif blue_blobs:  # 蓝色
            color_read_succed = 1
            largest_blob = max(blue_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "blue", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            block_cx = c_x
            block_cy = c_y
            
        elif green_blobs:  # 绿色
            color_read_succed = 1
            largest_blob = max(green_blobs, key=lambda b: b[2] * b[3])
            x, y, w, h, _, c_x, c_y = largest_blob
            
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 255), 2)
            cv2.drawMarker(frame, (cx, cy), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
            cv2.putText(frame, "green", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            block_cx = c_x
            block_cy = c_y
        print("move_status:", self.move_status)
        print("color_read_succed:", color_read_succed)
        # print("cx:", cx)
        # print("cy:", cy)
        # print("x:", x)
        # print("y:", y)
        # print("w:", w)
        # print("h:", h)

        #************************************************ 运动机械臂*************************************************************************************
        if color_read_succed==1 or (self.move_status==1):#识别到颜色
            if self.move_status==0:#第0阶段：机械臂寻找物块位置
                if(abs(block_cx-self.mid_block_cx)>3):
                    if block_cx > self.mid_block_cx:
                        self.move_x+=0.3
                    else:
                        self.move_x-=0.3
                if(abs(block_cy-self.mid_block_cy)>3):
                    if block_cy > self.mid_block_cy and self.move_y>80:
                        self.move_y-=0.3
                    else:
                        self.move_y+=0.3
                if abs(block_cy-self.mid_block_cy)<=3 and abs(block_cx-self.mid_block_cx)<=3: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt > 5:#计数10次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=1
                        self.cap_color_status=color_status
                else:
                    self.mid_block_cnt=0
                    kinematic.kinematics_move(self.move_x,self.move_y,70,10)
                time.sleep(0.1)

            elif self.move_status==1:#第1阶段：机械臂抓取物块
                self.move_status=2
                time.sleep(0.1)
                send_command("{#005P1000T1000!}")
                l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                sin=self.move_y/l
                cos=self.move_x/l
                self.move_x=(l+85+cy)*cos+cx
                self.move_y=(l+85+cy)*sin
                time.sleep(0.1)
                kinematic.kinematics_move(self.move_x,self.move_y,70,1000)#移动机械臂到物块上方
                time.sleep(1)
                kinematic.kinematics_move(self.move_x,self.move_y,25+cz,1000)#移动机械臂下移到物块
                time.sleep(1.2)
                send_command("{#005P1700T1000!}")#机械爪抓取物块
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x,self.move_y,120,1000)#移动机械臂抬起
                time.sleep(1.2)
                #机械臂旋转到要方向物块的指定位置
                self.move_x=100
                self.move_y=60
                kinematic.kinematics_move(self.move_x,self.move_y,120,1000)
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x,self.move_y,70,1000)
                time.sleep(1.2)
                self.mid_block_cnt=0

            elif self.move_status==2:#第2阶段：机械臂寻找放下物块的框框
                if(abs(block_cx-self.mid_block_cx)>5):
                    if block_cx > self.mid_block_cx and self.move_y>1:
                        self.move_y-=1
                    else:
                        self.move_y+=1
                if(abs(block_cy-self.mid_block_cy)>5):
                    if block_cy > self.mid_block_cy:
                        self.move_x-=1
                    else:
                        self.move_x+=1
                if abs(block_cy-self.mid_block_cy)<=5 and abs(block_cx-self.mid_block_cx)<=5: #寻找到物块，机械臂进入第二阶段
                    self.mid_block_cnt += 1
                    if self.mid_block_cnt>5:#计数5次对准物块，防止误差
                        self.mid_block_cnt=0
                        self.move_status=3
                        self.cap_color_status=color_status
                else:
                    self.mid_block_cnt=0
                    kinematic.kinematics_move(self.move_x,self.move_y,70,10)
                time.sleep(0.1)

            elif self.move_status==3:#第3阶段：机械臂放下物块并归位
                self.move_status=0
                l=math.sqrt(self.move_x*self.move_x+self.move_y*self.move_y)
                sin=self.move_y/l
                cos=self.move_x/l
                self.move_x=(l+85+cy+20)*cos - 50
                self.move_y=(l+85+cy+20)*sin
                time.sleep(0.1)
                kinematic.kinematics_move(self.move_x,self.move_y,70,1000)#移动机械臂到物块上方
                time.sleep(1)
                kinematic.kinematics_move(self.move_x,self.move_y,25+cz,1000)#移动机械臂下移到物块
                time.sleep(1.2)
                send_command("{#005P1000T1000!}")#机械爪放下物块
                time.sleep(1.2)
                kinematic.kinematics_move(self.move_x,self.move_y,70,1000)#移动机械臂抬起
                time.sleep(1.2)
                self.move_x=0#机械臂归位
                self.move_y=100
                kinematic.kinematics_move(self.move_x,self.move_y,70,1000)
                time.sleep(1.2)
                self.mid_block_cnt=0
                self.cap_color_status=0


def main():
    """主循环：处理图像和运行功能"""
    global run_app_status
    last_frame_time = time.time()
    
    # # 设置摄像头参数（根据实际需求调整）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
    # cap.set(cv2.CAP_PROP_FPS, 30)

    # send_command("$KMS:0,160,50,1000!\n")
    send_command("$KMS:0,100,70,1000!\n")
    # send_command("#openmv reset!")
    # send_command("$MV9!")

    # 初始化
    # line_Sort_Map2.init()
    colorPalletizer.init()
    # color_sort.init()
    # handle_uart_command()

    while True:
        # 读取摄像头帧
        ret, frame = cap.read()
        if not ret:
            print("无法获取图像")
            break
        
        # 计算帧率
        current_time = time.time()
        fps = 1 / (current_time - last_frame_time)
        last_frame_time = current_time
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # 根据状态运行对应功能
        # if run_app_status == 1:
        #     frame = car_Color_Trace.run(frame)
        # elif run_app_status == 2:
        #     frame = line_Map3.run(frame)
        # elif run_app_status == 3:
        #     frame = arm_Color_Trace.run(frame)
        # elif run_app_status == 4:
        #     frame = apriltag_Sort.run(frame, cx, cy, cz)
        # elif run_app_status == 5:
        #     frame = apriltag_Palletizer.run(frame, cx, cy, cz)
        # elif run_app_status == 6:
        #     frame = car_Color_Trace_Grasp.run(frame, cx, cy, cz)
        # elif run_app_status == 7:
        #     frame = line_Sort_Map1.run(frame, cx, cy, cz)
        # elif run_app_status == 8:
        #     frame = line_Sort_Map2.run(frame, cx, cy, cz)
        # elif run_app_status == 9:
        #     frame = face_Track.run(frame)
        # if run_app_status == 8:
            # line_Sort_Map2.run(cx, cy, cz)

        # 运行程序
        # line_Sort_Map2.run(frame, cx, cy, cz)
        colorPalletizer.run(frame, -20, -35, -100)
        # color_sort.run(frame, -20, -40, -100)
        # kinematic.kinematics_move(140, 90, -100, 1000)  # 移动机械臂下移到物块

        # 显示图像
        # cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 启动串口命令处理线程
    uart_thread = threading.Thread(target=handle_uart_command, daemon=True)
    uart_thread.start()

    # 初始化功能模块实例
    line_Sort_Map2 = LineSortMap2()
    kinematic = Kinematics()
    robot_move_cmd = RobotMoveCmd()
    colorPalletizer = ColorPalletizer()
    color_sort = ColorSort()

    # 启动主循环
    main()


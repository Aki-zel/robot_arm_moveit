#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PoseStamped
from robot_msgs.srv import Hand_Catch, Hand_CatchResponse
from tf.transformations import quaternion_from_euler, quaternion_multiply
from base_detect import BaseDetection
import os
import yaml


class ColorDetectServer(BaseDetection):
    def __init__(self, config):
        super(ColorDetectServer, self).__init__(config)
        self.bridge = CvBridge()
        self.service = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)
        # 读取颜色配置文件
        self.colors = config["colors"]
        rospy.loginfo("ColorDetectServer initialized")

    # 预处理图像，返回ROI区域和ROI区域的坐标
    def preprocess_image(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        x, y = 0, 0
        # 检查是否存在roi_min和roi_max的配置
        if "roi_min" not in self.color_threshold or "roi_max" not in self.color_threshold:
            rospy.logwarn("no ROI.")
            return cv_image, (x, y)

        # 定义ROI区域的HSV范围
        lower_hsv = np.array(self.color_threshold["roi_min"])
        upper_hsv = np.array(self.color_threshold["roi_max"])
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        # 进行形态学开运算(先腐蚀后膨胀)去除噪点
        kernel1 = np.ones((9, 9), np.uint8)# 定义一个9x9的卷积核，核大小一般为奇数，核越大效果越强
        morph_open = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        # 添加闭运算操作以合并连通域
        kernel2 = np.ones((5, 5), np.uint8)
        morph_close = cv2.morphologyEx(morph_open, cv2.MORPH_CLOSE, kernel2)

        # 查找roi轮廓
        contours, _ = cv2.findContours(
            morph_close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 绘制roi边界框
        if contours:
            largest_contour = max(contours, key=cv2.contourArea) # 找到面积最大的roi轮廓
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            roi = cv_image[y:y+h, x:x+w]  # 提取roi区域

        return roi, (x, y)

    # 颜色阈值处理
    def color_thresholding(self, cv_image, color_name, f, x=0, y=0):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])
        # 创建颜色掩码
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        image_path = os.path.join(current_work_dir, "2.png") 
        cv2.imwrite(image_path, mask) # 保存处理后的图像
        # 进行形态学开运算去除噪点
        kernel1 = np.ones((5, 5), np.uint8) 
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        image_path = os.path.join(current_work_dir, "3.png")
        cv2.imwrite(image_path, mask)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 检测抽屉把手时添加约束条件：长宽比大于4
        if f:
            # 滤除h>4*w的干扰轮廓
            contours = [cnt for cnt in contours if cv2.boundingRect(
                cnt)[2] >= 6*cv2.boundingRect(cnt)[3]]

        # 按图像y坐标对轮廓进行排序(从上到下)
        contours = sorted(
            contours, key=lambda contour: cv2.boundingRect(contour)[1])
        # 将检测到的目标信息保存到列表中
        objects_info = []
        for contour in contours:
            area = cv2.contourArea(contour)
            print(area) # 输出轮廓面积
            if self.area_threshold["min_area"] < area < self.area_threshold["max_area"]:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # 计算物体中心点和角度
                center_x, center_y = rect[0]
                angle = rect[2]
                cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                cv2.circle(cv_image, (int(center_x), int(center_y)),
                           5, (0, 0, 255), -1)

                center_x += x # 将检测到的ROI下的物体中心点坐标转换为相对于原图的坐标
                center_y += y
                objects_info.append({
                    'label': color_name,
                    'center_x': int(center_x),
                    'center_y': int(center_y),
                    'angle': angle
                })
        # 保存检测图像结果
        image_path = os.path.join(current_work_dir, "1.png") 
        cv2.imwrite(image_path, cv_image) 
        return objects_info

    # 颜色检测回调函数
    def handle_color_detection(self, request):
        response = Hand_CatchResponse()

        color_name = request.name
        if color_name not in self.colors:
            rospy.logerr(f"Color '{color_name}' not found in configuration")
            return Hand_CatchResponse(labels=[], positions=[])

        self.threshold = self.colors[color_name]
        self.color_threshold = self.threshold["color_threshold"]
        self.area_threshold = self.threshold["area_threshold"]
        f = False
        if request.run:
            if self.cv_image is not None:
                self.cv_image, (x, y) = self.preprocess_image(self.cv_image)
                # 检测抽屉把手时添加约束条件
                if color_name == "cabinet_handle":
                    f = True
                else:
                    f = False
                objects_info = self.color_thresholding(
                    self.cv_image, color_name,f, x, y)

                for obj in objects_info:
                    label = obj['label']
                    center_x = obj['center_x']
                    center_y = obj['center_y']
                    angle = obj['angle']

                    camera_xyz = self.getObject3DPosition(
                        center_x, center_y)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    world_position = self.tf_transform(
                        camera_xyz, [0, 0, np.deg2rad(angle)])
                    response.labels.append(label)
                    response.positions.append(world_position)

            else:
                rospy.logwarn("No image received yet.")
        return response


if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_work_dir, "config", "color.yaml")
    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file)
    try:
        rospy.init_node("color_detect_server")
        ColorDetectServer(config)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")

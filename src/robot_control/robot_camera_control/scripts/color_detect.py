#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from robot_msgs.srv import Hand_Catch, Hand_CatchResponse
from base_detect import BaseDetection
import os
import yaml


class ColorDetectServer(BaseDetection):
    def __init__(self, config):
        super(ColorDetectServer, self).__init__(config)
        self.bridge = CvBridge()
        self.service = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)

        # 读取整个颜色配置
        self.colors = config["colors"]

        rospy.loginfo("ColorDetectServer initialized")

    def preprocess_image(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 检查是否存在roi_min和roi_max的配置
        if "roi_min" not in self.color_threshold or "roi_max" not in self.color_threshold:
            rospy.logwarn("no ROI.")
            return cv_image
        
        # 定义ROI区域的HSV范围
        lower_hsv = np.array(self.color_threshold["roi_min"])
        upper_hsv = np.array(self.color_threshold["roi_max"])

        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        # 查找roi轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制roi边界框
        if contours:
            # 找到面积最大的roi轮廓
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            roi = cv_image[y:y+h, x:x+w]  # 提取roi区域

        return roi

    def color_thresholding(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])

        # 应用颜色阈值
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 按图像y坐标对轮廓进行排序(从上到下)
        contours = sorted(
            contours, key=lambda contour: cv2.boundingRect(contour)[1])

        objects_info = []

        for contour in contours:
            area = cv2.contourArea(contour)
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

                color = "detected_color"

                objects_info.append({
                    'label': color,
                    'center_x': int(center_x),
                    'center_y': int(center_y),
                    'angle': angle
                })
        
        # cv2.imshow("Color Detection", cv_image)
        # cv2.waitKey(0) 

        return objects_info

    def handle_color_detection(self, request):
        response = Hand_CatchResponse()

        color_name = request.color_name
        if color_name not in self.colors:
            rospy.logerr(f"Color '{color_name}' not found in configuration")
            return Hand_CatchResponse(labels=[], positions=[])
        
        self.threshold = self.colors[color_name]
        self.color_threshold = self.threshold["color_threshold"]
        self.area_threshold = self.threshold["area_threshold"]

        if request.run:
            if self.cv_image is not None:
                self.cv_image = self.preprocess_image(self.cv_image)
                objects_info = self.color_thresholding(
                    self.cv_image)

                for obj in objects_info:
                    label = obj['label']
                    center_x = obj['center_x']
                    center_y = obj['center_y']
                    angle = obj['angle']

                    camera_xyz = self.getObject3DPosition(
                        center_x, center_y)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    world_position = self.tf_transform(camera_xyz)
                    response.labels.append(label)
                    response.positions.append(world_position)
                    self.tf_broad(world_position)

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

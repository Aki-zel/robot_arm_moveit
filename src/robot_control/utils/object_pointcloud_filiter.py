#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import os
import yaml
from robot_msgs.srv import *
from preprocess import PreProcess
from object_detect import Yolo


current_work_dir = os.path.dirname(__file__)
config_path = os.path.join(current_work_dir, 'config.yaml')
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
model = Yolo(config)

def object_filiter(cloud_in):
    # 使用yolo模型检测
    color_image = model.cv_image
    model.Predicts(color_image)
    object_list = model.getFilteredObjects()  # 获取筛选后的目标信息列表

    if not object_list:
        rospy.logwarn("未检测到任何目标")
        return None
    
    obj = object_list[0]
    box = obj['box_coordinates']
    x_min, y_min, x_max, y_max = box
    # 获取左上角和右下角的相机坐标
    top_left = model.getObject3DPosition(int(x_min), int(y_min))
    bottom_right = model.getObject3DPosition(int(x_max), int(y_max))
    if top_left is None or bottom_right is None:
        rospy.logwarn("无法获取完整的深度信息")

    X_min, Y_min, Z_min = top_left
    X_max, Y_max, Z_max = bottom_right

    preprocess = PreProcess()
    filtered_cloud = preprocess.roi_filter(cloud_in, X_min, Y_min, X_max, Y_max)

    return filtered_cloud


if __name__ == '__main__':
    rospy.init_node('object_filiter')
    object_filiter()
    rospy.spin()

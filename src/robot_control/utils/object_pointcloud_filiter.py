#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import os
import yaml
from robot_msgs.srv import *
from preprocess import PreProcess
from yolo_detection import yolo
 

def object_filiter(cloud_in):
    current_work_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_work_dir, 'yolov5s_apple.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    model = yolo(config)
    filiter = PreProcess()

    color_image = model.cv_image
    model.Predicts(color_image)

    res = model.visual(color_image)  # 可视化检测结果
    cv2.imshow('res', res)
    
    object_list = model.getFilteredObjects()  # 获取筛选后的目标信息列表

    if object_list:
        for obj in object_list:
            box = obj['box_coordinates']
            x_min, y_min, x_max, y_max = box
            
            filtered_cloud = filiter.roi_filter(cloud_in, x_min, y_min, x_max, y_max)

    return filtered_cloud


if __name__ == '__main__':
    rospy.init_node('object_filiter')
    object_filiter()
    rospy.spin()

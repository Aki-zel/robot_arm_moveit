#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
import cv2
import os
import yaml
import fastdeploy.vision as vision
import fastdeploy as fd
from robot_msgs.srv import *


class yolo():
    def __init__(self, config):
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.result = None
        self.model = None
        self.config = config
        self.setOption(self.config['device'])  # 配置推理框架
        self.loadModel()
        self.cv_image = None
        self.depth_img = None
        self.camera_info = None
        self.setflag = 0

    def loadModel(self):
        if self.config['model_type'] == "ppyoloe":
            self.model = vision.detection.PPYOLOE(
                self.config['model_file'],
                self.config['params_file'],
                self.config['config_file'],
                runtime_option=self.option
            )
        elif self.config['model_type'] == "yolov5":
            self.model = vision.detection.YOLOv5(
                self.config['model_file'],
                self.config['params_file'],
                runtime_option=self.option
            )

    def setOption(self, type):
        self.option = fd.RuntimeOption()  # 存储运行时的选项，配置推理设备和后端
        # if type == "gpu":
        #     self.option.use_gpu()
        if type == "cpu":
            self.option.use_cpu()
        if type == "openvino":
            self.option.use_cpu()
            self.option.use_openvino_backend()  # 使用OpenVINO作为推理后端
        if type == "ort":
            self.option.use_cpu()
            self.option.use_ort_backend()
        if type == "trt":
            self.option.use_gpu()
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = current_work_dir+"/cache/model.trt"
        return self.option

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)

    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )
        return vis_im

    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result

    def getFilteredObjects(self):
        filtered = []  # 存储NMS筛选后的目标索引
        filtered_objects = []  # 存储最终目标信息
        object_info = {}  # 暂存单个目标信息
        score_threshold = self.config["threshold"]["confidence"]  # 获取置信度阈值
        iou = self.config["threshold"]["iou"]  # 获取IoU阈值
        # 使用OpenCV的非极大值抑制(NMS)来筛选检测框
        filtered = cv2.dnn.NMSBoxes(
            self.result.boxes, self.result.scores, score_threshold, iou)

        # 遍历筛选后的索引，获取对应的物体信息
        for i in filtered:
            object_info = {
                # 获取物体的类别标签
                'label': self.config['class_name'][self.result.label_ids[i]],
                'score': self.result.scores[i],  # 获取物体的置信度得分
                'box_coordinates': self.result.boxes[i]  # 获取物体的边界框坐标
            }
            filtered_objects.append(object_info)  # 将物体信息添加到结果列表中
        # 根据每个目标的置信度对目标列表进行排序
        sorted_objects = sorted(
            filtered_objects, key=lambda x: x['score'], reverse=True)

        return sorted_objects  # 返回筛选后的物体信息列表


if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_work_dir, 'yolov5s_apple.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    rospy.init_node("yolo_detection")
    model = yolo(config)
    rospy.spin()

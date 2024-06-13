#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time

import cv2
from base_detect import BaseDetection
import os
import yaml
import numpy as np
import fastdeploy.vision as vision
import fastdeploy as fd
from robot_msgs.srv import *
from cv_bridge import CvBridge
import rospy


class Yolo(BaseDetection):
    def __init__(self, config):
        super().__init__(config)
        self.result = None
        self.model = None
        self.cv_image = None
        self.setflag = 0
        self.setOption(self.config["device"])
        self.loadModel()

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
        self.option = fd.RuntimeOption()
        if type == "cpu":
            self.option.use_cpu()
        if type == "openvino":
            self.option.use_cpu()
            self.option.use_openvino_backend()
        if type == "ort":
            self.option.use_cpu()
            self.option.use_ort_backend()
        if type == "trt":
            self.option.use_gpu()
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = current_work_dir+"/cache/model.trt"
        return self.option

    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )
        return vis_im

    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result

    def getFilteredObjects(self):
        filtered = []
        filtered_objects = []
        object_info = {}
        score_threshold = self.config["threshold"]["confidence"]
        iou = self.config["threshold"]["iou"]
        filtered = cv2.dnn.NMSBoxes(
            self.result.boxes, self.result.scores, score_threshold, iou
        )
        for i in filtered:
            object_info = {
                "label": self.config["class_name"][self.result.label_ids[i]],
                "score": self.result.scores[i],
                "box_coordinates": self.result.boxes[i],
            }
            filtered_objects.append(object_info)
        sorted_objects = sorted(
            filtered_objects, key=lambda x: x["score"], reverse=True
        )
        return sorted_objects

def getObjCoordinate(request):
    global model
    labels = []
    positions = []

    run = request.run  # 获取请求中的标志位，判断是否执行检测
    respond = Hand_CatchResponse()  # 创建一个服务响应对象
    try:
        if run:
            color_image = model.cv_image
            # t_start = time.time()  # 开始计时
            model.Predicts(color_image)
            # t_end = time.time()  # 结束计时
            # print("预测时间" + str((t_end-t_start)*1000))  # 打印预测时间
            res = model.visual(color_image)  # 可视化检测结果
            respond.detect_image = CvBridge().cv2_to_imgmsg(res, encoding="bgr8")
            object_list = model.getFilteredObjects()  # 获取筛选后的目标信息列表
            # print("目标数量" + str(len(object_list)))
            tf_published = False  # 布尔变量，用于跟踪是否已经发布了TF坐标系
            if object_list:
                for obj in object_list:
                    label = obj["label"]
                    box_coords = obj["box_coordinates"]
                    # bottom_right = (box_coords[3] + 100, box_coords[2] + 100)
                    # top_left = (box_coords[1] + 100, box_coords[0] + 100)
                    ux = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
                    uy = int((box_coords[1] + box_coords[3]) / 2)  # 计算物体中心点y坐标
                    # grasp = model.grasp_gen.Predict(
                    #     color_image, depth_image, cam_info, top_left, bottom_right
                    # )
                    # 获取物体的三维坐标
                    camera_xyz = model.getObject3DPosition(ux, uy)
                    camera_xyz = np.round(
                        np.array(camera_xyz), 3).tolist()  # 转成3位小数
                    print(camera_xyz)
                    if camera_xyz[2] < 100.0 and camera_xyz[2] != 0:
                        world_pose = model.tf_transform(
                            camera_xyz, [0,0,0]
                        )  # 将目标物体从相机坐标系转换到世界坐标系
                        # model.tf_broad(camera_xyz)
                        positions.append(world_pose)
                        # positions.extend(camera_xyz)
                        labels.append(label)
                        # 仅发布第一个位置的TF坐标系
                        if not tf_published:  # 只发布置信度最高的目标TF
                            model.tf_broad(world_pose)
                            tf_published = True  # 将标志位设置为True，表示已发布TF坐标系
            respond.labels = labels
            respond.positions = positions
            print(respond.labels, respond.positions)
            return respond
    except Exception as r:
        rospy.loginfo("[ERROR] %s" % r)


if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = current_work_dir + "/config/config.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    rospy.init_node("camera_node")
    model = Yolo(config)

    service = rospy.Service("object_detect", Hand_Catch, getObjCoordinate)
    rospy.spin()

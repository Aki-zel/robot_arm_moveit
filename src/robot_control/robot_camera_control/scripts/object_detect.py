#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import yaml
import numpy as np
import fastdeploy.vision as vision
import fastdeploy as fd
from robot_msgs.srv import *
from transform import Transform


class yolo(Transform):
    def __init__(self, config):
        super(yolo, self).__init__()
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
                    label = obj['label']
                    box_coords = obj['box_coordinates']
                    ux = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
                    uy = int((box_coords[1] + box_coords[3]) / 2)  # 计算物体中心点y坐标
                    print("[INFO] detect success")
                    # 获取物体的三维坐标
                    camera_xyz = model.get_object_3d_position(ux, uy)
                    camera_xyz = np.round(
                        np.array(camera_xyz), 3).tolist()  # 转成3位小数
                    print(camera_xyz)
                    if camera_xyz[2] < 100.0 and camera_xyz[2] != 0:
                        world_pose = model.tf_transform(
                            camera_xyz)  # 将目标物体从相机坐标系转换到世界坐标系
                        if world_pose is not None:
                            world_position = [
                                world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z]
                        positions.extend(world_position)
                        labels.append(label)
                        # 仅发布第一个位置的TF坐标系
                        if len(positions) == 3 and not tf_published:  # 只发布置信度最高的目标TF
                            model.tf_broad(world_position)
                            tf_published = True  # 将标志位设置为True，表示已发布TF坐标系
            respond.labels = labels
            respond.positions = positions
            print(respond.labels, respond.positions)
            return respond
    except Exception as r:
        rospy.loginfo("[ERROR] %s" % r)


if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_work_dir, 'config/yolov5s_apple.yaml')
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    rospy.init_node("yolo_detect")
    model = yolo(config)
    service = rospy.Service("objection_detect", Hand_Catch, getObjCoordinate)
    rospy.spin()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import time
import cv2
import yaml
import rospy
import numpy as np
from base_detect import BaseDetection
import fastdeploy as fd
import fastdeploy.vision as vision
from robot_msgs.srv import Hand_Catch, Objection_Detect,Objection_DetectResponse,Hand_CatchResponse
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from threading import Lock


# 共享资源的线程锁
lock = Lock()

current_work_dir = os.path.dirname(__file__)
server = None

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
        model_type = self.config['model_type']
        if model_type == "ppyoloe":
            self.model = vision.detection.PPYOLOE(
                self.config['model_file'], self.config['params_file'],
                self.config['config_file'], runtime_option=self.option
            )
        elif model_type == "yolov5":
            self.model = vision.detection.YOLOv5(
                self.config['model_file'], self.config['params_file'],
                runtime_option=self.option
            )

    def setOption(self, device_type):
        self.option = fd.RuntimeOption()
        if device_type == "cpu":
            self.option.use_cpu()
        elif device_type == "openvino":
            self.option.use_cpu()
            self.option.use_openvino_backend()
        elif device_type == "ort":
            self.option.use_cpu()
            self.option.use_ort_backend()
        elif device_type == "gpu":
            self.option.use_gpu()
        elif device_type == "trt":
            self.option.use_gpu()
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = current_work_dir + "/cache/model.trt"
        return self.option

    def Predicts(self, img):
        """处理预测并返回结果"""
        self.result = self.model.predict(img)
        return self.result

    def visual(self, img):
        """可视化检测结果"""
        return vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )

    def getFilteredObjects(self):
        """过滤检测结果，获取有效目标"""
        filtered_objects = []
        score_threshold = self.config["threshold"]["confidence"]
        iou = self.config["threshold"]["iou"]
        filtered = cv2.dnn.NMSBoxes(self.result.boxes, self.result.scores, score_threshold, iou)
        
        for i in filtered:
            obj = {
                "label": self.config["class_name"][self.result.label_ids[i]],
                "score": self.result.scores[i],
                "box_coordinates": self.result.boxes[i],
            }
            filtered_objects.append(obj)
        
        # 按照y坐标排序
        return sorted(filtered_objects, key=lambda obj: obj["box_coordinates"][1])


def process_objects(object_list, filter_label=None):
    """处理并筛选特定标签的对象，返回其世界坐标"""
    global model
    labels, positions = [], []
    for obj in object_list:
        label = obj["label"]
        if filter_label and label != filter_label:
            continue

        box_coords = obj["box_coordinates"]
        ux, uy = int((box_coords[0] + box_coords[2]) / 2), int((box_coords[1] + box_coords[3]) / 2)
        camera_xyz = model.getObject3DPosition(ux, uy)
        camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 保留3位小数

        if camera_xyz[2] < 100.0 and camera_xyz[2] != 0:
            world_pose = model.tf_transform(camera_xyz)
            positions.append(world_pose)
            labels.append(label)
    return labels, positions


def getObjCoordinate(request):
    global model
    labels, positions = [], []
    run = request.run

    respond = Hand_CatchResponse()
    if run:
        try:
            color_image = model.cv_image
            t_start = time.time()
            model.Predicts(color_image)
            t_end = time.time()
            rospy.loginfo(f"预测时间: {(t_end - t_start) * 1000}ms")

            res = model.visual(color_image)  # 可视化检测结果
            image_path = os.path.join(current_work_dir, "object.png")
            cv2.imwrite(image_path, res)

            object_list = model.getFilteredObjects()
            if object_list:
                for obj in object_list:
                    if obj["label"] == request.name:
                        box_coords = obj["box_coordinates"]
                        ux, uy = int((box_coords[0] + box_coords[2]) / 2), int((box_coords[1] + box_coords[3]) / 2)
                        camera_xyz = model.getObject3DPosition(ux, uy)
                        camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                        print(box_coords,camera_xyz)
                        # 执行抓取检测
                        iteration_count = 0
                        while iteration_count < 15:
                            end_points, cloud = model.process_ros_image_data(box_coords, camera_xyz[2])
                            gg = model.get_grasps(end_points)
                            gg_k = model.collision_detection(gg, np.array(cloud.points))
                            gg_d = model.get_grasps_data(gg_k, 1)

                            if gg_d.grasp_group_array.size > 0 and gg_d.grasp_group_array[0, 0] > 0.95:
                                grasp = model.grasp_group_array_to_pose_stamped(gg_d.grasp_group_array)
                                for g in grasp:
                                    world_pose = model.tf_transform_pose(g)
                                    model.tf_broad(world_pose)
                                    positions.append(world_pose)
                                    labels.append(obj["label"])
                                break
                            iteration_count += 1

            respond.labels = labels
            respond.positions = positions
            print(respond.labels, respond.positions)
            return respond

        except Exception as e:
            rospy.loginfo(f"[ERROR] {e}")
    return respond


def realtime_detect_call_back(goal):
    global model
    run = goal.run
    feedback = Objection_DetectResponse()
    feedback.result = False
    world_pose = PoseStamped()
    feedback.position = world_pose

    if run:
        try:
            color_image = model.cv_image
            model.Predicts(color_image)
            res = model.visual(color_image)  # 可视化检测结果
            image_path = os.path.join(current_work_dir, "object.png")
            cv2.imwrite(image_path, res)

            object_list = model.getFilteredObjects()
            if object_list:
                for obj in object_list:
                    if obj["label"] == "plastic bottle":
                        feedback.result = True
                        box_coords = obj["box_coordinates"]
                        ux, uy = int((box_coords[0] + box_coords[2]) / 2), int((box_coords[1] + box_coords[3]) / 2)
                        camera_xyz = model.getObject3DPosition(ux, uy)
                        camera_xyz = np.round(np.array(camera_xyz), 3).tolist()

                        if camera_xyz[2] != 0:
                            world_pose = model.tf_transform_name(camera_xyz, model.config['robot']['base_frame_id'])
                            feedback.position = world_pose
                            if 0.36 <= camera_xyz[2] < 0.52 and 220 <= ux <= 420:
                                feedback.success = True
        except Exception as e:
            rospy.loginfo(f"[ERROR] {e}")
    return feedback


if __name__ == '__main__':
    # 加载配置文件
    config_path = os.path.join(current_work_dir, "config/config1.yaml")
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)

    rospy.init_node("object_detect_server")
    model = Yolo(config)
    service = rospy.Service("object_detect", Hand_Catch, getObjCoordinate)
    server = rospy.Service("object_realtime_detect", Objection_Detect, realtime_detect_call_back)
    rospy.loginfo("服务启动")
    rospy.spin()

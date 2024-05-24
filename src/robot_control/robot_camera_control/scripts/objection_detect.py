#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import yaml
import numpy as np
import fastdeploy.vision as vision
import fastdeploy as fd
import threading
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_geometry_msgs
from robot_msgs.srv import *


class yolo:
    def __init__(self, config):
        self.result = None
        self.model = None
        self.config = config
        self.setOption(self.config['device']) # 配置推理框架
        self.loadModel()
        self.cv_image = None
        self.depth_img = None
        self.camera_info = None
        self.setflag = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # 创建TF广播

    def loadModel(self):
        if self.config['model_type']=="ppyoloe":
            self.model = vision.detection.PPYOLOE(
                    self.config['model_file'],
                    self.config['params_file'],
                    self.config['config_file'],
                    runtime_option=self.option
            )
        elif self.config['model_type']=="yolov5":
            self.model = vision.detection.YOLOv5(
                self.config['model_file'],
                self.config['params_file'],
                runtime_option=self.option
            )

    def setOption(self,type):
        self.option = fd.RuntimeOption() # 存储运行时的选项，配置推理设备和后端
        # if type == "gpu":
        #     self.option.use_gpu()
        if type == "cpu":
            self.option.use_cpu()
        if type == "openvino":
            self.option.use_cpu()
            self.option.use_openvino_backend() # 使用OpenVINO作为推理后端
        if type== "ort": 
            self.option.use_cpu()
            self.option.use_ort_backend()  
        if type== "trt":
            self.option.use_gpu()
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = current_work_dir+"/cache/model.trt"
        return self.option
    
    def depthImageCallback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def cameraInfoCallback(self, msg):
        self.camera_info = msg
    
    def image_callback(self,msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")   
        except Exception as e:
            print(e)
    
    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )
        return vis_im

    def getObject3DPosition(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None

        # 获取相机固有参数
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        depth_value = self.depth_img[y, x]

        # 将像素坐标转换为三维点
        X = (x - cx) * depth_value / fx / 1000
        Y = (y - cy) * depth_value / fy / 1000
        Z = depth_value/1000

        return [X, Y, Z]
    
    def tf_transform(self, position):
        x, y, z = position
        camera_point = PoseStamped()
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        camera_point.pose.orientation.w = 1

        try:
            # 使用tf2将机械臂摄像头坐标系转换到base_link坐标系
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1))
            world_point = tf2_geometry_msgs.do_transform_pose(camera_point, transform)
            if world_point is not None:
                rospy.loginfo("World point: %s", world_point)
                return world_point
            else:
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None    

    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result
    
    def tf_broad(self, position): 
        tfs = TransformStamped() # 创建广播数据
        tfs.header.frame_id = "base_link"  # 参考坐标系
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "object"  # 目标坐标系
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation= position.pose.orientation
        # 发布tf变换
        self.tf_broadcaster.sendTransform(tfs)

    def getFilteredObjects(self):
        filtered=[] # 存储NMS筛选后的目标索引
        filtered_objects = [] # 存储最终目标信息
        object_info = {} # 暂存单个目标信息
        score_threshold=self.config["threshold"]["confidence"] # 获取置信度阈值
        iou=self.config["threshold"]["iou"] # 获取IoU阈值
        # 使用OpenCV的非极大值抑制(NMS)来筛选检测框
        filtered=cv2.dnn.NMSBoxes(self.result.boxes, self.result.scores, score_threshold, iou)
        
        # 遍历筛选后的索引，获取对应的物体信息
        for i in filtered:
            object_info = {
                'label': self.config['class_name'][self.result.label_ids[i]], # 获取物体的类别标签
                'score': self.result.scores[i], # 获取物体的置信度得分
                'box_coordinates': self.result.boxes[i] # 获取物体的边界框坐标
            }
            filtered_objects.append(object_info) # 将物体信息添加到结果列表中
        # 根据每个目标的置信度对目标列表进行排序
        sorted_objects = sorted(filtered_objects, key=lambda x: x['score'], reverse=True)
        
        return sorted_objects # 返回筛选后的物体信息列表
    
def getObjCoordinate(request):
    global model
    labels = []
    positions = []

    run = request.run # 获取请求中的标志位，判断是否执行检测
    respond = Hand_CatchResponse() # 创建一个服务响应对象
    try:
        if run:
            color_image = model.cv_image
            t_start = time.time()  # 开始计时
            model.Predicts(color_image)
            t_end = time.time()  # 结束计时
            res = model.visual(color_image) # 可视化检测结果
            respond.detect_image = CvBridge().cv2_to_imgmsg(res, encoding="bgr8")
            print("预测时间" + str((t_end-t_start)*1000)) # 打印预测时间
            object_list = model.getFilteredObjects() # 获取筛选后的目标信息列表
            print("目标数量" + str(len(object_list)))
            if object_list:
                for obj in object_list:
                    label = obj['label']
                    box_coords = obj['box_coordinates']
                    ux = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
                    uy = int((box_coords[1] + box_coords[3]) / 2)  # 计算物体中心点y坐标
                    print("[INFO] detect success")
                    # 获取物体的三维坐标
                    camera_xyz = model.getObject3DPosition(ux,uy)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 转成3位小数
                    if camera_xyz[2] < 100.0 and camera_xyz[2]!=0:
                        world_pose = model.tf_transform(camera_xyz) # 将目标物体从相机坐标系转换到世界坐标系
                        # model.tf_broad(camera_xyz)
                        if world_pose is not None:
                            world_position = [world_pose.pose.position.x, world_pose.pose.position.y, world_pose.pose.position.z]
                        positions.extend(world_position)
                    # positions.extend(camera_xyz)
                        labels.append(label)
            respond.labels = labels
            respond.positions = positions
            print (respond.labels, respond.positions)
            return respond
    except Exception as r:
        rospy.loginfo("[ERROR] %s"%r)

if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = current_work_dir + '/config/yolov5s_apple.yaml'
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    rospy.init_node("camera_node")  
    model = yolo(config)
    rospy.Subscriber('/camera/color/image_raw', Image, model.image_callback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',
                     Image, model.depthImageCallback)
    rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',
                     CameraInfo, model.cameraInfoCallback)
    service=rospy.Service("objection_detect", Hand_Catch, getObjCoordinate)
    rospy.spin()
    cv2.destroyAllWindows()
    
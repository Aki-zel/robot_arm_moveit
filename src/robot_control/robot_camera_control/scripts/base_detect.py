#!/usr/bin python3
# -*- coding: utf-8 -*-


import cv2
import numpy as np
import rospy
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf
import math
import os
import sys
import open3d as o3d
import argparse
import torch

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped,Point, Quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply
from scipy.spatial.transform import Rotation as R
import fastdeploy as fd
import fastdeploy.vision as vision

from graspnetAPI import GraspGroup
from grasp_net.models.graspnet  import GraspNet, pred_decode
from grasp_net.utils.collision_detector import ModelFreeCollisionDetector
from grasp_net.utils.data_utils import CameraInfo as GraspCameraInfo, create_point_cloud_from_depth_image


current_work_dir = os.path.dirname(__file__)
class BaseDetection:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.depth_img = None
        self.camera_info = None
        self.cv_image = None
        self.result = None
        self.model = None
        self.colors = config["colors"]
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.count = 0
        self.num_point=config["num_point"]
        self.num_view=config["num_view"]
        self.collision_thresh=config["collision_thresh"]
        self.voxel_size=config["voxel_size"]
        self.net=self.get_net()
        self.initTopic()
        
    def get_net(self):
        # 初始化模型
        net = GraspNet(input_feature_dim=0, num_view=self.num_view, num_angle=12, num_depth=4,
                cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01,0.02,0.03,0.04], is_training=False)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        net.to(device)
        # 加载模型检查点
        checkpoint = torch.load(self.config["gmodel_file"])
        net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        print("loaded checkpoint %s (epoch: %d)"%(self.config["gmodel_file"], start_epoch))
        net.eval()
        return net
   
    def initTopic(self):
        rospy.Subscriber(self.config['camera']['color_topic'],
                         Image, self.image_callback)
        rospy.Subscriber(self.config['camera']['depth_topic'],
                         Image, self.depth_image_cb)
        rospy.Subscriber(self.config['camera']['info_topic'],
                         CameraInfo, self.camera_info_cb)
        # rospy.wait_for_service("/call_task")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def depth_image_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='32FC1')

    def camera_info_cb(self, msg):
        self.camera_info = msg

    
    # 将相机坐标系中的位置转换为世界坐标系中的位置(相对于配置文件的机器人基坐标系)
    def tf_transform(self, position, grasp=[0, 0, 0]):
        x, y, z = position
        camera_point = PoseStamped()
        camera_point.header.frame_id = self.config['camera']['frame_id']
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        quaternion = tf.quaternion_from_euler(grasp[0], grasp[1], grasp[2])
        camera_point.pose.orientation.w = quaternion[3]
        camera_point.pose.orientation.x = quaternion[0]
        camera_point.pose.orientation.y = quaternion[1]
        camera_point.pose.orientation.z = quaternion[2]
        try:
            transform = self.tf_buffer.lookup_transform(
                self.config['robot']['base_frame_id'],
                self.config['camera']['frame_id'],
                rospy.Time(0),
                rospy.Duration(1),
            )
            world_point = tf2_geometry_msgs.do_transform_pose(
                camera_point, transform)
            if world_point is not None:
                # rospy.loginfo("World point: %s", world_point.pose)
                # self.tf_broad(world_point)
                return world_point
            else:
                return None
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None
    def tf_transform_pose(self, position):
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.config['robot']['base_frame_id'],
                    self.config['camera']['frame_id'],
                    rospy.Time(0),
                    rospy.Duration(1),
                )
                world_point = tf2_geometry_msgs.do_transform_pose(
                    position, transform)
                if world_point is not None:
                    # rospy.loginfo("World point: %s", world_point.pose)
                    # self.tf_broad(world_point)
                    return world_point
                else:
                    return None
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn("Exception while transforming: %s", e)
                return None


    def tf_broad(self, position):
        tfs = TransformStamped()
        tfs.header.frame_id = self.config['robot']['base_frame_id']
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "object"+str(self.count)
        self.count = self.count+1
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation = position.pose.orientation
        self.tf_broadcaster.sendTransform(tfs)

    def tf_broad_name(self, position, name):
        tfs = TransformStamped()
        tfs.header.frame_id = name
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "objet_name"
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation = position.pose.orientation
        self.tf_broadcaster.sendTransform(tfs)
    
    # 抓取姿态检测部分
        # 根据目标最小外接矩形的角度旋转的姿态(平面抓取)
    def grasp_transform_pose(self, world_position, angle, frame_id="base_link_rm"):
        # 将相机坐标系中的位置转换为世界坐标系中的位置
        if world_position is None:
            rospy.logwarn("Transformation to world position failed.")
            return None
        
        # 创建PoseStamped消息
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()  # 设置时间戳
        pose.header.frame_id = frame_id  # 设置坐标系id
        # 设置位置
        pose.pose.position.x = world_position.pose.position.x
        pose.pose.position.y = world_position.pose.position.y
        pose.pose.position.z = world_position.pose.position.z
        # 末端工具默认朝下时的姿态（初始四元数）
   
        # rospy.loginfo("angle %f",angle)
        yaw = math.radians(angle)  # 将yaw角转换成弧度
        initial_quaternion = quaternion_from_euler(0,math.pi,0, axes='sxyz')  # 初始四元数，表示末端工具默认朝下
        # rospy.loginfo(f'Final Quaternion: {initial_quaternion}')
        # 计算绕 Z 轴旋转的四元数
        rotation_quaternion = quaternion_from_euler(0, 0, yaw) # 参数为roll、pitch、yaw
        # 合成最终的四元数（将初始四元数与旋转四元数相乘）
        final_quaternion = quaternion_multiply(initial_quaternion, rotation_quaternion)
        # rospy.loginfo(f'Final Quaternion: {final_quaternion}')
        # 设置姿态
        pose.pose.orientation.x = final_quaternion[0]
        pose.pose.orientation.y = final_quaternion[1]
        pose.pose.orientation.z = final_quaternion[2]
        pose.pose.orientation.w = final_quaternion[3]

        return pose
    
    def create_workspace_mask(self, depth_image, box, expansion=55):
        """
        创建一个矩形形状的工作空间掩码，控制感兴趣区域的范围。
        
        :param depth_image: 输入的深度图像（二维 numpy 数组）
        :param box: 一个包含 (x_min, y_min, x_max, y_max) 的元组或列表，
                    表示YOLO识别出的边界框。
        :param expansion: 扩展边界框的像素大小（默认为5像素），扩展值可根据需要调整。

        :return: 一个布尔值掩码，表示有效区域
        """
        # 提取 box 中的 x_min, y_min, x_max, y_max
        x_min, y_min, x_max, y_max = box
        
        # 扩展 box 边界
        x_min = max(int(x_min) - expansion, 0)  # 防止超出图像范围
        y_min = max(int(y_min) - expansion, 0)
        x_max = min(int(x_max) + expansion, depth_image.shape[1])  # 防止超出图像范围
        y_max = min(int(y_max) + expansion, depth_image.shape[0])

        # 获取深度图像的尺寸
        height, width = depth_image.shape
        
        # 创建一个全为 False 的布尔数组
        mask = np.zeros((height, width), dtype=bool)
        
        # 设置扩展后的工作空间范围内的区域为 True
        mask[y_min:y_max, x_min:x_max] = True  # 设置扩展后的区域为 True
        
        return mask


    def process_ros_image_data(self,box,filter_depth):
        if self.cv_image is not None and self.depth_img is not None and self.camera_info is not None:
            # 转换为OpenCV格式
            color = np.array(self.cv_image, dtype=np.float32) / 255.0
            depth = np.array(self.depth_img, dtype=np.float32)

            # 使用元数据生成点云
            intrinsic_matrix = np.array(self.camera_info.K).reshape(3, 3)  # 将K字段转换为NumPy数组并重塑为3x3矩阵
            camera = GraspCameraInfo(1280.0, 720.0, intrinsic_matrix[0, 0], intrinsic_matrix[1, 1], intrinsic_matrix[0, 2], intrinsic_matrix[1, 2], 1000)
            cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
            workspace_mask = self.create_workspace_mask(depth, box)
            # 获取有效点
            mask = (workspace_mask & (depth > 0) & (depth < filter_depth*1000+100))
            cloud_masked = cloud[mask]
            color_masked = color[mask]

            # 随机采样点云
            if len(cloud_masked) >= self.num_point:
                idxs = np.random.choice(len(cloud_masked), self.num_point, replace=False)
            else:
                idxs1 = np.arange(len(cloud_masked))
                idxs2 = np.random.choice(len(cloud_masked), self.num_point-len(cloud_masked), replace=True)
                idxs = np.concatenate([idxs1, idxs2], axis=0)
            cloud_sampled = cloud_masked[idxs]
            color_sampled = color_masked[idxs]

            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
            cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
            end_points = dict()
            cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
            device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            cloud_sampled = cloud_sampled.to(device)
            end_points['point_clouds'] = cloud_sampled
            end_points['cloud_colors'] = color_sampled

        return end_points, cloud
    
    def get_grasps(self, end_points):
        with torch.no_grad():
            end_points = self.net(end_points)
            grasp_preds = pred_decode(end_points)
        gg_array = grasp_preds[0].detach().cpu().numpy()
        gg = GraspGroup(gg_array)

        return gg

    def collision_detection(self,gg, cloud):
        mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=self.voxel_size)
        collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=self.collision_thresh)
        gg = gg[~collision_mask]
        return gg
    
    def grasp_group_array_to_pose_stamped(self,grasp_group_array):
        poses = []
        
        # 假设 grasp_group_array 的形状是 (n, 17)
        for i in range(grasp_group_array.shape[0]):
            # 提取平移向量 (x, y, z)
            translation = grasp_group_array[i, 13:16]  # 形状是 (3,)
            
            # 提取旋转矩阵并转换为四元数
            rotation_matrix = grasp_group_array[i, 4:13].reshape(3, 3)  # 形状是 (3, 3)
            # 交换旋转矩阵的 x 轴和 z 轴
            rotation_matrix[:, [0, 2]] = rotation_matrix[:, [2, 0]]
            
            r = R.from_matrix(rotation_matrix)
            quaternion = r.as_quat()  # 返回四元数 [x, y, z, w]
            
            # 创建 PoseStamped 消息
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()  # 当前时间戳
            pose_stamped.header.frame_id = "camera_color_optical_frame"  # 坐标系名称，依据需求设置
            
            # 创建位置 (Point)
            pose_stamped.pose.position = Point(*translation)  # 传入平移向量
            
            # 创建姿态 (Quaternion)
            pose_stamped.pose.orientation = Quaternion(*quaternion)  # 传入四元数
            # 将 PoseStamped 消息添加到列表中
            poses.append(pose_stamped)
        
        return poses
    
    def get_grasps_data(self,gg, num):
        gg.nms()
        gg.sort_by_score()
        gg = gg[:num]

        return gg
    
    # 颜色检测部分


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
    
    # yolo视觉部分
    
    def loadYoloModel(self):
        self.setOption(self.config["device"])
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
        if type == "gpu":
            self.option.use_gpu()
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
    
    def getObject3DPosition(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        x = int(round(x))
        y = int(round(y))
        depth_value = self.depth_img[y, x]
        if depth_value == 0:
            rospy.logerr("未检测到深度信息!!!!!")
            X=Y=Z=0
        else:
            Z = depth_value / 1000
            X = (x - cx) * Z / fx 
            Y = (y - cy) * Z / fy 

        return [X, Y, Z]
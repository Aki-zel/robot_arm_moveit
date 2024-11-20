#!/usr/bin python3
# -*- coding: utf-8 -*-

# Import standard libraries
import os
import threading
import time
import yaml
# Import custom modules
from base_detect import BaseDetection

# Import third-party libraries
import open3d as o3d
import cv2
import numpy as np
import fastdeploy as fd
import fastdeploy.vision as vision

# Import ROS-related libraries
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from robot_msgs.srv import Hand_Catch, Objection_Detect,Objection_DetectResponse,Hand_CatchResponse
import actionlib
from tf.transformations import quaternion_from_euler, quaternion_multiply
from std_msgs.msg import String




current_work_dir = os.path.dirname(__file__)
class CameraControlCenter(BaseDetection):
    def __init__(self, config):
        super().__init__(config)
        self.services = {}  # 存储服务句柄
        self.publishers = {}  # 存储话题发布器句柄
        self.loadYoloModel()
    
    def create_ros_entities(self):
        """
        创建所需的 ROS 话题和服务
        """
        # 创建服务
        self.services['object_detect'] = rospy.Service("object_detect", Hand_Catch, self.handle_object_detect)
        self.services['color_detect'] = rospy.Service("color_detect", Hand_Catch, self.handle_color_detect)

        # 创建话题
        self.publishers['status'] = rospy.Publisher("camera_status", String, queue_size=10)
        self.publishers['realtime_detect'] = rospy.Publisher("realtime_detect", String, queue_size=10)

        rospy.loginfo("ROS topics and services created successfully.")

    def handle_object_detect(self, req):
        """
        服务回调：处理 object_detect 请求
        """
        rospy.loginfo("Object detection service called.")
        feedback = Objection_DetectResponse()
        labels, positions = [], []
        # 根据需求处理 req，并返回响应
        if req.run:
            try:
                # 对模型进行处理
                color_image = self.cv_image
                t_start = time.time()
                self.Predicts(color_image)
                t_end = time.time()
                rospy.loginfo(f"预测时间: {(t_end - t_start) * 1000}ms")

                res = self.visual(color_image)  # 可视化检测结果
                image_path = os.path.join(current_work_dir, "object.png")
                cv2.imwrite(image_path, res)
                object_list = self.getFilteredObjects()
                if object_list:
                    for obj in object_list:
                        if obj["labels"] == req.name or req.name=="":
                            box_coords = obj["box_coordinates"]
                            ux, uy = int((box_coords[0] + box_coords[2]) / 2), int((box_coords[1] + box_coords[3]) / 2)
                            camera_xyz = self.getObject3DPosition(ux, uy)
                            camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                            if req.isgrasp:
                                iteration_count = 0
                                while iteration_count < 15:
                                    end_points, cloud = self.process_ros_image_data(box_coords, camera_xyz[2])
                                    gg = self.get_grasps(end_points)
                                    gg_k = self.collision_detection(gg, np.array(cloud.points))
                                    gg_d = self.get_grasps_data(gg_k, 1)

                                    if gg_d.grasp_group_array.size > 0 and gg_d.grasp_group_array[0, 0] > 0.95:
                                        grasp = self.grasp_group_array_to_pose_stamped(gg_d.grasp_group_array)
                                        for g in grasp:
                                            world_pose = self.tf_transform_pose(g)
                                            self.tf_broad(world_pose)
                                            positions.append(world_pose)
                                            labels.append(obj["label"])
                                        break
                                    iteration_count +=1
                            else:
                                world_pose = self.tf_transform(camera_xyz)
                                positions.append(world_pose)
                                labels.append(obj["label"])

                feedback.labels = labels
                feedback.positions = positions
                print(feedback.labels, feedback.positions)
                return feedback
            except Exception as e:
                rospy.loginfo(f"[ERROR] {e}")

        
  
        return feedback

    def handle_color_detect(self, req):
        """
        服务回调：处理 color_detect 请求
        """
        rospy.loginfo("Color detection service called.")
        # 根据需求处理 req，并返回响应
        response = Hand_CatchResponse()
        color_name = req.name
        if color_name not in self.colors:
            rospy.logerr(f"Color '{color_name}' not found in configuration")
            return Hand_CatchResponse(labels=[], positions=[])
        self.threshold = self.colors[color_name]
        self.color_threshold = self.threshold["color_threshold"]
        self.area_threshold = self.threshold["area_threshold"]
        f = False
        if req.run:
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

    def publish_status(self, status_message):
        """
        发布状态消息到话题
        """
        if 'status' in self.publishers:
            self.publishers['status'].publish(status_message)
            rospy.loginfo(f"Published status: {status_message}")
        else:
            rospy.logwarn("Status topic not initialized.")

    def publish_realtime_detect(self):
        """
        在单独的线程中将数据连续发布到“realtime_detect”主题
        """
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Example data (can be replaced with actual detection result)
            realtime_data = "Detection data"
            if 'realtime_detect' in self.publishers:
                self.publishers['realtime_detect'].publish(realtime_data)
                rospy.loginfo(f"Published realtime detect data: {realtime_data}")
            rate.sleep()

    def start_realtime_detect_thread(self):
        """
        在单独的线程中启动实时检测发布
        """
        detection_thread = threading.Thread(target=self.publish_realtime_detect)
        detection_thread.daemon = True
        detection_thread.start()

if __name__ == '__main__':
    rospy.init_node("camera_control_center", anonymous=True)
    config_path = os.path.join(current_work_dir, "config", "config1.yaml")
    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file)
    CameraControlCenter(config)

    rospy.spin()
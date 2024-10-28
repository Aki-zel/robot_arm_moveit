#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tf
from tf.transformations import quaternion_from_euler, quaternion_multiply
import math
# from graspany_detect import GraspGenerator


class BaseDetection:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.depth_img = None
        self.camera_info = None
        self.cv_image = None
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # self.grasp_gen = GraspGenerator(config["gmodel_file"], True)
        # self.grasp_gen.load_model()
        self.count = 0
        self.initTopic()

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
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def camera_info_cb(self, msg):
        self.camera_info = msg

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

    # 将相机坐标系中的位置转换为世界坐标系中的位置(相对于小车)
    def tf_transform_name(self, position, name="base_link"):
        x, y, z = position
        camera_point = PoseStamped()
        camera_point.header.frame_id = self.config['camera']['frame_id']
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        quaternion = tf.quaternion_from_euler(0, 0, 0)
        camera_point.pose.orientation.w = quaternion[3]
        camera_point.pose.orientation.x = quaternion[0]
        camera_point.pose.orientation.y = quaternion[1]
        camera_point.pose.orientation.z = quaternion[2]
        try:
            transform = self.tf_buffer.lookup_transform(
                name,
                self.config['camera']['frame_id'],
                rospy.Time(0),
                rospy.Duration(1),
            )
            world_point = tf2_geometry_msgs.do_transform_pose(
                camera_point, transform)
            if world_point is not None:
                # rospy.loginfo("World point: %s", world_point.pose)
                self.tf_broad_name(world_point, name)
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

    # 根据目标最小外接矩形的角度旋转的姿态(平面抓取)
    def transform_pose(self, world_position, angle, frame_id="base_link_rm"):
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

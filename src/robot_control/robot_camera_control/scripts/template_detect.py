#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from grasp_detect import GraspGenerator
import tf.transformations as tf


class TemplateDetect:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher(
            "template_detect_image", Image, queue_size=1)
        rospy.Subscriber(
            "/camera/color/image_raw/", Image, self.image_callback
        )  # 订阅相机图像
        rospy.Subscriber("/image_template", Image, self.template_cb)  # 订阅模版图像
        self.template_image = None
        # 图像与世界坐标系间tf坐标转换，订阅相机内参和深度图像用于获取物体三维坐标
        self.depth_img = None
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_cb
        )
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info",
            CameraInfo,
            self.camera_info_cb,
        )
        # 发布目标位姿信息
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()  # 创建TF广播器
        self.object_position_pub = rospy.Publisher(
            "object_position", PoseStamped, queue_size=10
        )

        self.grasp_deter = GraspGenerator("/home/ydf/Documents/rwm_moveit/src/robot_control/robot_camera_control/scripts/weights/model_cornell_0.98", True)
        self.grasp_deter.load_model()

    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # 如果模板图像尚未加载，则不执行后续操作
        if self.template_image is None:
            return

        # 进行模板匹配
        res = cv2.matchTemplate(
            cv_image, self.template_image, cv2.TM_CCOEFF_NORMED)
        # 获取匹配结果中的最大值和其对应的位置
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        template_height, template_width = self.template_image.shape[:2]
        # 计算边界
        top = max_loc[1]
        bottom = top + template_height
        left = max_loc[0]
        right = left + template_width
        # 计算模板在原始图像中的中心坐标
        template_center_x = max_loc[0] + self.template_image.shape[1] // 2
        template_center_y = max_loc[1] + self.template_image.shape[0] // 2
        bottom_right = (
            min(480, bottom + 100),
            min(640, right + 100),
        )
        top_left = (
            max(0, top - 100),
            max(left - 100, 0),
        )
        grasp = self.grasp_deter.Predict(
            cv_image, self.depth_img, self.camera_info, top_left, bottom_right
        )
        # 输出模板中心坐标
        rospy.loginfo(
            "Template center coordinates: ({}, {})".format(
                template_center_x, template_center_y
            )
        )
        if (len(grasp)!=0):
            rospy.loginfo(
                "Grasp center point: ({}, {})".format(grasp[0], grasp[1]))
            self.template_image = None
            # 将物体中心点像素坐标转换为世界坐标系下的三维坐标
            object_position = self.get_object_3d_position(int(grasp[0]), int(grasp[1]))
            if object_position is not None:
                rospy.loginfo(
                    "Object position in world coordinates: {}".format(
                        object_position)
                )
                self.tf_transform(object_position, grasp)
            else:
                rospy.logwarn("Unable to determine object position.")

    def template_cb(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.template_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.template_image = cv2.cvtColor(
                self.template_image, cv2.COLOR_BGR2RGB)
            # 获取原始图像的宽度和高度
            original_height, original_width, _ = self.template_image.shape
            # 计算新的宽度和高度
            new_width = int(original_width / 451 * 640)
            new_height = int(original_height / 419 * 480)
            # 缩放图像
            self.template_image = cv2.resize(
                self.template_image, (new_width, new_height)
            )
        except CvBridgeError as e:
            print(e)

    def depth_image_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def get_object_3d_position(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None
        # 获取相机固有参数
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        depth_value = self.depth_img[y, x]
        if depth_value == 0:
            return None

        # 将像素坐标转换为三维点
        x = (x - cx) * depth_value / fx / 1000
        y = (y - cy) * depth_value / fy / 1000
        z = depth_value / 1000

        return x, y, z

    def tf_transform(self, position, grasp):
        x, y, z = position
        camera_point = PoseStamped()
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        quaternion = tf.quaternion_from_euler(grasp[2], grasp[3], grasp[4])
        camera_point.pose.orientation.w = quaternion[3]
        camera_point.pose.orientation.x = quaternion[0]
        camera_point.pose.orientation.y = quaternion[1]
        camera_point.pose.orientation.z = quaternion[2]

        try:
            # 使用tf2将机械臂摄像头坐标系转换到base_link坐标系
            transform = self.tf_buffer.lookup_transform(
                "base_link_rm",
                "camera_color_optical_frame",
                rospy.Time(0),
                rospy.Duration(1),
            )
            world_point = tf2_geometry_msgs.do_transform_pose(
                camera_point, transform)
            if world_point is not None:
                rospy.loginfo("World point: %s", world_point)
                self.object_position_pub.publish(world_point)
                self.tf_broad(world_point)
            else:
                return None
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None

    # 发布物体的TF坐标系

    def tf_broad(self, position):
        tfs = TransformStamped()  # 创建广播数据
        tfs.header.frame_id = "base_link_rm"  # 参考坐标系
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "object"  # 目标坐标系
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation = position.pose.orientation
        # 发布tf变换
        self.tf_broadcaster.sendTransform(tfs)


if __name__ == "__main__":
    try:
        # 初始化ros节点
        rospy.init_node("template_detect")
        rospy.loginfo("Starting template_detect node")
        find_object = TemplateDetect()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

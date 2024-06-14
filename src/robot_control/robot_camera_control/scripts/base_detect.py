#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf.transformations as tf
# from graspany_detect import GraspGenerator


class BaseDetection:
    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.depth_img = None
        self.camera_info = None
        self.cv_image=None
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # self.grasp_gen = GraspGenerator(config["gmodel_file"], True)
        # self.grasp_gen.load_model()
        self.initTopic()
        print("111")

    def initTopic(self):
        rospy.Subscriber(self.config['camera']['color_topic'],
                         Image, self.image_callback)
        rospy.Subscriber(self.config['camera']['depth_topic'],
                         Image, self.depth_image_cb)
        rospy.Subscriber(self.config['camera']['info_topic'],
                         CameraInfo, self.camera_info_cb)

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
        depth_value = self.depth_img[y, x]
        if depth_value == 0:
            rospy.logerr("未检测到深度信息!!!!!")
        X = (x - cx) * depth_value / fx / 1000
        Y = (y - cy) * depth_value / fy / 1000
        Z = depth_value / 1000

        return [X, Y, Z]

    def tf_transform(self, position, grasp = [0,0,0]):
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
                rospy.loginfo("World point: %s", world_point)
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
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation = position.pose.orientation
        self.tf_broadcaster.sendTransform(tfs)

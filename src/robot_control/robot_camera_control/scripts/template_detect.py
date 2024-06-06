#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import yaml
from base_detect import BaseDetection
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from graspany_detect import GraspGenerator
import tf.transformations as tf


class TemplateDetect(BaseDetection):
    def __init__(self, config):
        super().__init__(config)
        self.template_image = None
        self.image_pub = rospy.Publisher("template_detect_image", Image, queue_size=1)
        rospy.Subscriber("/image_template", Image, self.template_cb)
        self.object_position_pub = rospy.Publisher("object_position", PoseStamped, queue_size=10)

    def image_callback(self, msg):
        super().image_callback(msg)
        if self.template_image is None:
            return
        res = cv2.matchTemplate(
            self.cv_image, self.template_image, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        template_height, template_width = self.template_image.shape[:2]
        top = max_loc[1]
        bottom = top + template_height
        left = max_loc[0]
        right = left + template_width
        template_center_x = max_loc[0] + self.template_image.shape[1] // 2
        template_center_y = max_loc[1] + self.template_image.shape[0] // 2
        # bottom_right = (
        #     min(480, bottom + 100),
        #     min(640, right + 100),
        # )
        # top_left = (
        #     max(0, top - 100),
        #     max(left - 100, 0),
        # )
        # grasp = self.grasp_gen.Predict(
        #     self.cv_image, self.depth_img, self.camera_info, top_left, bottom_right
        # )
        # rospy.loginfo(
        #     "Template center coordinates: ({}, {})".format(
        #         template_center_x, template_center_y
        #     )
        # )
        # if (len(grasp) != 0):
        #     rospy.loginfo(
        #         "Grasp center point: ({}, {})".format(grasp[0], grasp[1]))
        self.template_image = None
        object_position = self.getObject3DPosition(
            int(template_center_x), int(template_center_y))
        # object_position[2]=0.15
        # object_position[0]+=0.1
        if object_position is not None:
            rospy.loginfo(
                "Object position in world coordinates: {}".format(
                    object_position)
            )
            word_pose=self.tf_transform(object_position, [0,0,0])
            self.object_position_pub.publish(word_pose)
        else:
            rospy.logwarn("Unable to determine object position.")

    def template_cb(self, data):
        try:
            self.template_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.template_image = cv2.cvtColor(
                self.template_image, cv2.COLOR_BGR2RGB)
            original_height, original_width, _ = self.template_image.shape
            target_height = 200
            target_width = int(
                original_width * (target_height / original_height))
            self.template_image = cv2.resize(
                self.template_image, (target_width, target_height))
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    try:
        current_work_dir = os.path.dirname(__file__)
        config_path = current_work_dir + "/config/config.yaml"
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)
        # 初始化ros节点
        rospy.init_node("template_detect")
        rospy.loginfo("Starting template_detect node")
        find_object = TemplateDetect(config)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

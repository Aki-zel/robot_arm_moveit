#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from transform import Transform

class TemplateDetect(Transform):
    def __init__(self):    
        super(TemplateDetect, self).__init__()
        self.image_pub = rospy.Publisher("template_detect_image", Image, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw/", Image, self.image_callback) # 订阅相机图像
        rospy.Subscriber("/image_template", Image, self.template_cb) # 订阅模版图像
        self.template_image = None
        # 发布目标位姿信息
        self.object_position_pub = rospy.Publisher("object_position", PoseStamped, queue_size=10)

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
        res = cv2.matchTemplate(cv_image, self.template_image, cv2.TM_CCOEFF_NORMED)
        # 获取匹配结果中的最大值和其对应的位置
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # 计算模板在原始图像中的中心坐标
        template_center_x = max_loc[0] + self.template_image.shape[1] // 2
        template_center_y = max_loc[1] + self.template_image.shape[0] // 2

        # 输出模板中心坐标
        rospy.loginfo("Template center coordinates: ({}, {})".format(template_center_x, template_center_y))

        # 在原始图像中绘制模板的位置
        cv2.rectangle(cv_image, max_loc, (max_loc[0] + self.template_image.shape[1], max_loc[1] + self.template_image.shape[0]), (0, 255, 0), 2)

        # 发布处理后的图像
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        self.template_image=None
        # 将物体中心点像素坐标转换为世界坐标系下的三维坐标
        object_position = self.get_object_3d_position(template_center_x, template_center_y)
        if object_position is not None:
            rospy.loginfo("Object position in world coordinates: {}".format(object_position))
            self.tf_transform(object_position)
            self.tf_broad(object_position)
        else:
            rospy.logwarn("Unable to determine object position.")

    def template_cb(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            self.template_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.template_image = cv2.cvtColor(self.template_image, cv2.COLOR_BGR2RGB)
            # 获取原始图像的宽度和高度
            original_height, original_width, _ = self.template_image.shape
            # 计算新的宽度和高度
            new_width = int(original_width /451*1280)
            new_height = int(original_height /419*720)
            # 缩放图像
            self.template_image = cv2.resize(self.template_image, (new_width, new_height))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("template_detect")
        rospy.loginfo("Starting template_detect node")
        find_object = TemplateDetect()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down cv_bridge_test node.")
        cv2.destroyAllWindows()

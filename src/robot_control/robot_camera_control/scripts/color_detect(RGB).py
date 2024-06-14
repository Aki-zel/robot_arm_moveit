import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import PoseStamped
from robot_msgs.srv import Hand_Catch, Hand_CatchResponse
from tf.transformations import quaternion_from_euler, quaternion_multiply
from base_detect import BaseDetection
import os
import yaml


class ColorDetectServer(BaseDetection):
    def __init__(self, config):
        super(ColorDetectServer, self).__init__(config)
        self.bridge = CvBridge()
        self.service = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)
        rospy.loginfo("ColorDetectServer initialized")

    def color_thresholding(self, cv_image):
        # 将图像转换为HSV颜色空间
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 定义颜色的阈值范围（在HSV颜色空间中）
        lower_red1 = np.array([0, 80, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 80, 100])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([40, 80, 100])
        upper_green = np.array([80, 255, 255])

        lower_blue = np.array([90, 80, 100])
        upper_blue = np.array([130, 255, 255])

        # 对于每种颜色，进行颜色分割
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # 合并三种颜色的掩码
        mask = mask_red + mask_green + mask_blue

        # 寻找物体轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects_info = []  # 存储检测到的物体信息

        for contour in contours:
            area = cv2.contourArea(contour)
            if 5000 < area < 50000:
                # 计算最小外接矩形
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                # 绘制轮廓
                cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)

                # 计算物体中心点和角度
                center_x, center_y = rect[0]
                angle = rect[2]

                # 确定物体颜色
                color = ""
                if mask_red[int(center_y), int(center_x)] > 0:
                    color = "red"
                elif mask_green[int(center_y), int(center_x)] > 0:
                    color = "green"
                elif mask_blue[int(center_y), int(center_x)] > 0:
                    color = "blue"

                # 添加检测到的物体信息到列表中
                objects_info.append({
                    'label': color,
                    'center_x': int(center_x),
                    'center_y': int(center_y),
                    'angle': angle
                })

        return objects_info, cv_image

    def handle_color_detection(self, request):
        response = Hand_CatchResponse()

        if request.run:
            if self.cv_image is not None:
                objects_info, processed_image = self.color_thresholding(
                    self.cv_image)

                tf_published = False  # 布尔变量，用于跟踪是否已经发布了TF坐标系
                for obj in objects_info:
                    label = obj['label']
                    center_x = obj['center_x']
                    center_y = obj['center_y']
                    angle = obj['angle']

                    # 获取物体的三维坐标
                    camera_xyz = self.getObject3DPosition(
                        center_x, center_y)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()

                    # 从相机坐标系到世界坐标系的转换
                    world_position = self.tf_transform(camera_xyz)

                    # 创建PoseStamped消息
                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "camera_color_optical_frame"

                    # 设置位置
                    pose.pose.position.x = world_position.pose.position.x
                    pose.pose.position.y = world_position.pose.position.y
                    pose.pose.position.z = world_position.pose.position.z

                    # 末端工具默认朝下时的姿态
                    initial_quaternion = [0, 1, 0, 0] # 初始四元数
                    # 计算绕 Z 轴旋转的四元数
                    rotation_quaternion = quaternion_from_euler(
                        0, 0, np.deg2rad(angle))
                    # 合成四元数
                    final_quaternion = quaternion_multiply(
                        initial_quaternion, rotation_quaternion)
                    # 设置姿态
                    pose.pose.orientation.x = final_quaternion[0]
                    pose.pose.orientation.y = final_quaternion[1]
                    pose.pose.orientation.z = final_quaternion[2]
                    pose.pose.orientation.w = final_quaternion[3]

                    response.labels.append(label)
                    response.positions.append(pose)
                    # 仅发布第一个位置的TF坐标系
                    if len(camera_xyz) == 3 and not tf_published:  # 只发布置信度最高的目标TF
                        self.tf_broad(world_position)
                        tf_published = True  # 将标志位设置为True，表示已发布TF坐标系

                try:
                    response.detect_image = self.bridge.cv2_to_imgmsg(
                        processed_image, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr("Error converting image to message: %s" % e)
                else:
                    rospy.loginfo("Image converted to message successfully")
            else:
                rospy.logwarn("No image received yet.")
        return response


if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = current_work_dir + "/config/config.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    try:
        rospy.init_node("color_detect_server")
        ColorDetectServer(config)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")

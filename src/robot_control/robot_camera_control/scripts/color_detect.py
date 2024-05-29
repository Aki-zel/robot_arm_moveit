import rospy
import cv2
from cv_bridge import CvBridgeError
import numpy as np
from sensor_msgs.msg import Image
from transform import Transform
from robot_msgs.srv import *


class ColorDetectServer(Transform):
    def __init__(self):
        super(ColorDetectServer, self).__init__()
        rospy.Subscriber("/camera/color/image_raw", Image,
                         self.image_callback)  # 订阅相机图像
        self.service = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(e)

    def color_thresholding(self, cv_image):
        # 定义三种颜色的阈值范围（在 RGB 颜色空间中）
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([80, 80, 255])

        lower_green = np.array([0, 100, 0])
        upper_green = np.array([80, 255, 80])

        lower_blue = np.array([100, 0, 0])
        upper_blue = np.array([255, 80, 80])

        # 转换图像到 HSV 颜色空间
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 对于每种颜色，进行颜色分割
        mask_red = cv2.inRange(cv_image, lower_red, upper_red)
        mask_green = cv2.inRange(cv_image, lower_green, upper_green)
        mask_blue = cv2.inRange(cv_image, lower_blue, upper_blue)

        # 合并三种颜色的掩码
        mask = mask_red + mask_green + mask_blue

        # 寻找物体轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 保存检测到的物体的位置和颜色信息
        labels = []
        positions = []

        # 对每个轮廓进行处理
        for contour in contours:
            # 计算轮廓的矩形边界
            x, y, w, h = cv2.boundingRect(contour)
            # 计算物体中心点坐标
            center_x = x + w // 2
            center_y = y + h // 2

            # 根据物体中心点的颜色确定物体颜色
            color = ""
            if mask_red[center_y, center_x] > 0:
                color = "red"
            elif mask_green[center_y, center_x] > 0:
                color = "green"
            elif mask_blue[center_y, center_x] > 0:
                color = "blue"

            # 添加检测到的物体信息到列表中
            labels.append(color)
            positions.append((center_x, center_y))

            # 在图像上标记检测到的物体中心
            detect_image = cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)

        return labels, positions, detect_image

    def handle_color_detection(self, request):
         # 创建响应对象
        run = request.run
        response = Hand_CatchResponse()

        if run:
            cv_image = self.bridge.imgmsg_to_cv2(self.cv_image, "bgr8")
            labels, positions, detect_image = self.color_thresholding(cv_image)
            response.labels = labels
            response.positions = positions
            # 发布处理后的图像
            try:
                response.detect_image = self.bridge.cv2_to_imgmsg(detect_image, "bgr8")
            except CvBridgeError as e:
                rospy.logerr(e)
            return response


if __name__ == '__main__':
    try:
        rospy.init_node("color_detect_server")
        rospy.loginfo("Starting color_detect_server node")
        color_detect_server = ColorDetectServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")

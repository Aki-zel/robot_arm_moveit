#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_msgs.srv import Get_Board_State, Get_Board_StateResponse, Hand_Catch, Hand_CatchResponse, Hand_CatchRequest
from base_detect import BaseDetection
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_multiply
import yaml
import os


class ChessboardDetection(BaseDetection):
    def __init__(self, config):
        super(ChessboardDetection, self).__init__(config)
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher(
            "chessboard_image", Image, queue_size=1)
        self.bridge = CvBridge()
        self.service1 = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)
        self.service2 = rospy.Service(
            "chessboard_detect", Get_Board_State, self.handle_chessboard_detection)
        # 读取颜色配置文件
        self.colors = config["colors"]
        rospy.loginfo("ChessboardDetectServer initialized")

    # 颜色阈值处理

    def color_thresholding(self, cv_image, color_name):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])
        # 创建颜色掩码
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        # 进行形态学开运算去除噪点
        kernel1 = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 按图像y坐标对轮廓进行排序(从上到下)
        contours = sorted(
            contours, key=lambda contour: cv2.boundingRect(contour)[1])
        # 将检测到的目标信息保存到列表中
        objects_info = []
        for contour in contours:
            area = cv2.contourArea(contour)
            print(area)  # 输出轮廓面积
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

                objects_info.append({
                    'label': color_name,
                    'center_x': int(center_x),
                    'center_y': int(center_y),
                    'angle': angle
                })
        return objects_info

    # 颜色检测回调函数
    def handle_color_detection(self, request):
        response = Hand_CatchResponse()

        color_name = request.name
        if color_name not in self.colors:
            rospy.logerr(f"Color '{color_name}' not found in configuration")
            return Hand_CatchResponse(labels=[], positions=[])

        self.threshold = self.colors[color_name]
        self.color_threshold = self.threshold["color_threshold"]
        self.area_threshold = self.threshold["area_threshold"]
        f = False
        if request.run:
            if self.cv_image is not None:
                objects_info = self.color_thresholding(
                    self.cv_image, color_name)

                for obj in objects_info:
                    label = obj['label']
                    center_x = obj['center_x']
                    center_y = obj['center_y']
                    angle = obj['angle']

                    camera_xyz = self.getObject3DPosition(
                        center_x, center_y)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    world_position = self.tf_transform(camera_xyz,[0, 0, np.deg2rad(angle)])
        
                    response.labels.append(label)
                    response.positions.append(world_position)

            else:
                rospy.logwarn("No image received yet.")
        return response

    def check_for_piece(self, cell_img):
        # 将输入图像转换为 HSV 色域
        hsv_image = cv2.cvtColor(cell_img, cv2.COLOR_BGR2HSV)

        # 蓝色阈值
        lower_blue = np.array(self.colors['blue']['color_threshold']['lower'])
        upper_blue = np.array(self.colors['blue']['color_threshold']['upper'])

        # 橙色阈值
        lower_orange = np.array(self.colors['orange']['color_threshold']['lower'])
        upper_orange = np.array(self.colors['orange']['color_threshold']['upper'])

        # 创建蓝色掩码
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        # 进行形态学开运算去除噪点
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        
        # 创建橙色掩码
        mask_orange = cv2.inRange(hsv_image, lower_orange, upper_orange)
        mask_orange = cv2.morphologyEx(mask_orange, cv2.MORPH_OPEN, kernel)

        # 查找蓝色掩码中的轮廓
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_detected = False
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if self.colors['blue']['area_threshold']['min_area'] < area < self.colors['blue']['area_threshold']['max_area']:
                blue_detected = True
                break

        # 查找橙色掩码中的轮廓
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_detected = False
        for contour in contours_orange:
            area = cv2.contourArea(contour)
            if self.colors['orange']['area_threshold']['min_area'] < area < self.colors['orange']['area_threshold']['max_area']:
                orange_detected = True
                break

        # 返回检测结果
        if blue_detected:
            return 1
        elif orange_detected:
            return 0
        else:
            return -1

    def get_rotated_rect_corners(self, center, size, angle):
            w, h = size
            points = np.array([
                [-w / 2, -h / 2],
                [w / 2, -h / 2],
                [w / 2, h / 2],
                [-w / 2, h / 2]
            ])
            R = cv2.getRotationMatrix2D((0, 0), -angle, 1)
            points = np.dot(points, R[:, :2].T) + center
            return np.int0(points)

    def crop_polygon(self, cv_image, corners):
        if corners.shape != (4, 2):
            raise ValueError("Corners array must be of shape (4, 2)")

        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [corners], 255)

        if np.all(mask == 0):
            raise ValueError("Mask is not being filled correctly. Check corners array.")

        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        x_min = np.min(corners[:, 0])
        y_min = np.min(corners[:, 1])
        x_max = np.max(corners[:, 0])
        y_max = np.max(corners[:, 1])

        cropped_image = result[y_min:y_max, x_min:x_max]
        return cropped_image

    def handle_chessboard_detection(self, request):
        response = Get_Board_StateResponse()
        if request.run:

            if self.cv_image is None:
                rospy.logwarn("尚未接收到图像。")
                response.board = []
                response.positions = []
                response.round = 0
                return response

            # 预处理图像
            cv_image = self.cv_image
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)
            kernel = np.ones((3, 3), np.uint8)
            closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 寻找四边形
            squares = []
            for contour in contours:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                if len(approx) == 4 and cv2.isContourConvex(approx):
                    squares.append(approx)

            positions = []
            board = []
            round = 0
            
            if len(squares) > 0:
                # 合并轮廓并计算凸包
                merged_contour = np.vstack(squares).squeeze()
                hull = cv2.convexHull(merged_contour)
                cv_image = cv2.drawContours(cv_image, [hull], -1, (0, 255, 0), 3)

                # 计算最小外接矩形
                rect = cv2.minAreaRect(hull)
                center, (width, height), angle = rect
                center = np.array(center)

                # 计算棋盘格的中心点
                board_size = (3, 3)
                cell_width = width / board_size[0]
                cell_height = height / board_size[1]

                grid_centers = []
                for i in range(board_size[0]):
                    for j in range(board_size[1]):
                        x = (i - 1) * cell_width + cell_width / 2
                        y = (j - 1) * cell_height + cell_height / 2
                        grid_centers.append((x, y))

                tx = cell_width / 2
                ty = cell_height / 2

                translated_centers = []
                for (x, y) in grid_centers:
                    x_trans = x - tx
                    y_trans = y - ty
                    translated_centers.append((x_trans, y_trans))

                angle_rad = np.deg2rad(angle)
                rotated_centers = []
                for (x, y) in translated_centers:
                    x_rot = (x * np.cos(angle_rad) - y * np.sin(angle_rad)) + center[0]
                    y_rot = (x * np.sin(angle_rad) + y * np.cos(angle_rad)) + center[1]
                    rotated_centers.append((x_rot, y_rot))

                # 计算逆旋转变换矩阵
                R_inv = cv2.getRotationMatrix2D(center, angle_rad, 1)

                # 将旋转后的坐标变换回原图坐标
                original_centers = []
                for (x_rot, y_rot) in rotated_centers:
                    x_orig, y_orig = np.dot(R_inv[:, :2], [x_rot - center[0], y_rot - center[1]]) + center
                    original_centers.append((x_orig, y_orig))

                # 将旋转后的棋盘格中心点转换为世界坐标
                for (cx, cy) in original_centers:
                    # cv2.circle(cv_image, (int(cx), int(cy)), 5, (255, 0, 0), -1)
                    # cv2.putText(cv_image, f"({int(cx)}, {int(cy)})", (int(cx), int(cy) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
                    # 获取图像坐标的3D位置（在相机坐标系中）
                    camera_xyz = self.getObject3DPosition(cx, cy)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 四舍五入到3位小数
                    # 将相机坐标系中的位置转换为世界坐标系中的位置
                    world_position = self.tf_transform(camera_xyz)
                    self.tf_broad(world_position)
                    # 将世界坐标添加到positions数组
                    positions.append(world_position)

                # 输出旋转后的棋盘格中心点的原图坐标
                for idx, (cx, cy) in enumerate(original_centers):
                    print(f"棋盘格中心 {idx + 1}: ({cx:.2f}, {cy:.2f})")

                # 遍历每个棋盘格并裁剪
                for i in range(board_size[0]):
                    for j in range(board_size[1]):
                        cx, cy = rotated_centers[i * board_size[1] + j]
                        corners = self.get_rotated_rect_corners((cx, cy), (cell_width, cell_height), angle)
                        
                        cv_image_display = cv_image.copy()
                        cv2.polylines(cv_image_display, [corners], isClosed=True, color=(255, 0, 0), thickness=2)
                        for (x, y) in corners:
                            cv2.circle(cv_image_display, (int(x), int(y)), 5, (0, 0, 255), -1)

                        cropped_image = self.crop_polygon(cv_image, corners)
                        # cv2.imwrite(f"{i}_{j}.jpg", cropped_image)

                        if cropped_image.size > 0:
                            has_piece = self.check_for_piece(cropped_image)
                            if has_piece == 1 or has_piece == 0:
                                round += 1
                            board.append(has_piece)

                try:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                except CvBridgeError as e:
                    rospy.logerr(f"CvBridgeError: {e}")

            else:
                rospy.loginfo("未找到棋盘格")

            response.board = board
            response.positions = positions
            response.round = round

        return response



if __name__ == '__main__':
    current_work_dir = os.path.dirname(__file__)
    config_path = os.path.join(current_work_dir, "config", "color.yaml")
    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file)
    try:
        # 初始化ROS节点
        rospy.init_node("chessboard_detect")
        rospy.loginfo("Starting chessboard node")
        ChessboardDetection(config)
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down chessboard_detect node.")
        cv2.destroyAllWindows()

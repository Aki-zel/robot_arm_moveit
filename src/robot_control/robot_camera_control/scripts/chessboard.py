#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_msgs.srv import Get_Board_State, Get_Board_StateResponse, Hand_Catch, Hand_CatchResponse, Hand_CatchRequest
from base_detect import BaseDetection
import yaml
import os


class ChessboardDetection(BaseDetection):
    def __init__(self, config):
        super(ChessboardDetection, self).__init__(config)
        # 发布检测后的棋盘图像
        self.image_pub = rospy.Publisher(
            "chessboard_image", Image, queue_size=1)
        self.service1 = rospy.Service(
            "color_detect", Hand_Catch, self.handle_color_detection)
        self.service2 = rospy.Service(
            "chessboard_detect", Get_Board_State, self.handle_chessboard_detection)
        # 读取颜色配置文件
        self.colors = config["colors"]
        rospy.loginfo("ChessboardDetectServer initialized")

    # 颜色阈值处理检测棋盘外棋子
    def color_thresholding(self, cv_image, color_name):
        # 检测棋盘区域并获取掩码
        success, _, _, _, _, hull = self.chessboard_detect(cv_image)
        if success:
            # 创建棋盘区域掩码
            chessboard_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            # cv2.drawContours(chessboard_mask, [hull], -1, 255, -1)
            chessboard_mask_inv = cv2.bitwise_not(chessboard_mask)
        else:
            chessboard_mask_inv = None

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        image_path = os.path.join(current_work_dir, "1.png") 
        cv2.imwrite(image_path, cv_image) # 保存处理后的图像
        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])
        # 创建颜色掩码
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        image_path = os.path.join(current_work_dir, "2.png") 
        cv2.imwrite(image_path, mask) # 保存处理后的图像
        # 进行形态学开运算去除噪点
        # kernel1 = np.ones((5, 5), np.uint8)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        # image_path = os.path.join(current_work_dir, "3.png") 
        # cv2.imwrite(image_path, mask) # 保存处理后的图像
        # 应用棋盘区域掩码，排除棋盘区域
        if chessboard_mask_inv is not None:
            mask = cv2.bitwise_and(mask, mask, mask=chessboard_mask_inv)

        # 查找轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 按图像y坐标对轮廓进行排序(从上到下)
        contours = sorted(
            contours, key=lambda contour: cv2.boundingRect(contour)[0],reverse=True)

        # 将检测到的目标信息保存到列表中
        objects_info = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # epsilon = 0.02 * cv2.arcLength(contour, True)
            # approx = cv2.approxPolyDP(contour, epsilon, True)
            print(area)  # 输出轮廓面积
            if self.area_threshold["min_area"] < area < self.area_threshold["max_area"] :
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                # 计算物体中心点和角度
                center_x, center_y = rect[0]
                width, height = rect[1]
                angle = rect[2]
                # 根据检测框宽高调整角度，使得夹取间距较短
                if width <= height:  # 默认左下边为宽，顺时针转为正方向
                    angle = angle
                else:
                    angle = angle-90
                # cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
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
                    if camera_xyz == [0,0,0]:
                        continue
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    world_position = self.tf_transform(camera_xyz)
                    pose = self.transform_pose(world_position, angle)

                    response.labels.append(label)
                    response.positions.append(pose)

            else:
                rospy.logwarn("No image received yet.")
        return response

    # 颜色检测
    def detect_color(self, hsv_image, color_name):
        # 获取颜色的阈值范围
        lower = np.array(self.colors[color_name]['color_threshold']['lower'])
        upper = np.array(self.colors[color_name]['color_threshold']['upper'])

        # 创建颜色掩码并进行形态学开运算
        mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))

        # 查找掩码中的轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.colors[color_name]['area_threshold']['min_area'] < area < self.colors[color_name]['area_threshold']['max_area']:
                return True
        return False

    # 检测棋盘格内棋子
    def check_for_piece(self, cell_img):
        # 将输入图像转换为 HSV 色域
        hsv_image = cv2.cvtColor(cell_img, cv2.COLOR_BGR2HSV)

        # 检查己方
        player_detected = self.detect_color(hsv_image, 'green')

        # 检查敌方
        opponent_detected = self.detect_color(hsv_image, 'orange')

        # 返回检测结果
        if player_detected:
            return 1  # 检测到己方
        elif opponent_detected:
            return 0  # 检测到敌方
        else:
            return -1  # 未检测到颜色

    # 获取旋转矩形的四个顶点坐标
    def get_rotated_rect_corners(self, center, size, angle):
        w, h = size
        points = np.array([  # 未旋转时的四个顶点坐标
            [-w / 2, -h / 2],
            [w / 2, -h / 2],
            [w / 2, h / 2],
            [-w / 2, h / 2]
        ])
        R = cv2.getRotationMatrix2D((0, 0), -angle, 1)  # 创建旋转矩阵
        points = np.dot(points, R[:, :2].T) + center  # 将顶点坐标转换为相对于中心点的坐标
        return np.int0(points)

    # 裁剪多边形
    def crop_polygon(self, cv_image, corners):
        if corners.shape != (4, 2):  # 检查数组形状
            raise ValueError("Corners array must be of shape (4, 2)")

        # 转换角点数据类型为 int32
        corners = np.int32(corners)

        mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [corners], 255)

        if np.all(mask == 0):
            raise ValueError(
                "Mask is not being filled correctly. Check corners array.")

        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        x_min = np.min(corners[:, 0])
        y_min = np.min(corners[:, 1])
        x_max = np.max(corners[:, 0])
        y_max = np.max(corners[:, 1])

        cropped_image = result[y_min:y_max, x_min:x_max]
        return cropped_image
    def crop_circles(self,image, x,y, diameter):
        """
        从给定的图像中裁剪出指定直径的圆形区域，并保存这些区域。
        
        :param image: 要裁剪的原始图像 (numpy array)。
        :param rotated_centers: 圆心的坐标列表，格式为 [(x1, y1), (x2, y2), ...]。
        :param diameter: 圆的直径 (int)。
        :return: 裁剪出的圆形图片列表 (list of numpy arrays)。
        """
        radius = int(diameter / 2)

        
        # 创建一个空白的掩码
        mask = np.zeros_like(image, dtype=np.uint8)

        # 在掩码上绘制圆形
        cv2.circle(mask, (int(x), int(y)), radius, (255, 255, 255), thickness=-1)

        # 使用掩码裁剪图像
        circular_region = cv2.bitwise_and(image, mask)

        # 获取圆形区域的边界
        x_min = max(int(x - radius), 0)
        y_min = max(int(y - radius), 0)
        x_max = min(int(x + radius), image.shape[1])
        y_max = min(int(y + radius), image.shape[0])

        # 裁剪圆形区域的边界框
        cropped_image = circular_region[y_min:y_max, x_min:x_max]

        return cropped_image
    # 棋盘检测
    def chessboard_detect(self, cv_image):
        # 预处理图像
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 去噪
        dst = cv2.fastNlMeansDenoising(gray, None, 25, 7, 21)
        # cv2.imwrite("dst.jpg", dst)
        blur = cv2.GaussianBlur(dst, (5, 5), 0)
        # 使用拉普拉斯算子计算图像的二阶导数
        laplacian = cv2.Laplacian(blur, cv2.CV_64F)
        # 将拉普拉斯算子的结果转化为与原图像相同的数据类型
        laplacian = np.uint8(np.absolute(laplacian))
        # 锐化图像：原图像 - 拉普拉斯算子结果
        sharpened = cv2.subtract(blur, laplacian)
        # cv2.imwrite("sharpened.jpg", sharpened)
        edges = cv2.Canny(sharpened, 50, 150)
        # cv2.imwrite("edges.jpg", edges)
        kernel = np.ones((5, 5), np.uint8)
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        # cv2.imwrite("closed.jpg", closed)
        contours, _ = cv2.findContours(
            closed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 寻找四边形
        squares = []
        for contour in contours:
            epsilon = 0.02 * \
                cv2.arcLength(contour, True)  # 轮廓的近似程度，设为轮廓周长的1%,越小越近似
            approx = cv2.approxPolyDP(contour, epsilon, True)  # 轮廓的近似多边形
            area = cv2.contourArea(approx)  # 计算轮廓面积
            # 满足轮廓面积条件(顶点数量为4并且大于棋子的轮廓面积)，则将轮廓添加到列表中
            if len(approx) == 4 and cv2.isContourConvex(approx) and area > 1000:
                squares.append(approx)

        if len(squares) > 0:
            # 合并轮廓并计算凸包
            merged_contour = np.vstack(squares).squeeze()
            hull = cv2.convexHull(merged_contour)  # 计算凸包,凸包是包含轮廓所有点的最小凸多边形
            # cv_image = cv2.drawContours(
            #     cv_image, [hull], -1, (255, 255, 0), 3)
            
            # 计算最小外接矩形
            rect = cv2.minAreaRect(hull)
            center, (width, height), angle = rect
            center = np.array(center)
            # 获取矩形的四个顶点
            box = cv2.boxPoints(rect)
            box = np.int0(box)  # 将顶点转换为整数
            # cv_image = cv2.drawContours(
            #     cv_image, [box], -1, (0, 0, 255), 3)
            # 计算凸包的边长
            hull_edges = []
            for i in range(len(hull)):
                pt1 = hull[i][0]
                pt2 = hull[(i + 1) % len(hull)][0]  # 相邻点，注意取模处理循环边界
                edge_length = np.linalg.norm(pt1 - pt2)  # 计算两点之间的欧几里得距离
                hull_edges.append(edge_length)
            
            return True, center, width, height, angle, hull
        else:
            return False, None, None, None, None, None
    def line_equation(self,p1, p2):
        """计算两点之间的直线方程 Ax + By + C = 0"""
        A = p2[1] - p1[1]
        B = p1[0] - p2[0]
        C = A * p1[0] + B * p1[1]
        return A, B, -C

    def intersection(self,line1, line2):
        """计算两条直线的交点"""
        A1, B1, C1 = line1
        A2, B2, C2 = line2
        det = A1 * B2 - A2 * B1
        if det == 0:
            return None  # 直线平行
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        return np.array([x, y])

    def handle_chessboard_detection(self, request):
        response = Get_Board_StateResponse()
        positions = []
        board = []
        round = 0
        board_size = (10, 10)

        if request.run:
            if self.cv_image is None:
                rospy.logwarn("尚未接收到图像。")
                response.board = []
                response.positions = []
                response.round = 0
                return response
            
            # 调用棋盘检测函数
            detected, center, width, height, angle, box = self.chessboard_detect(
                self.cv_image)
            grid_centers1=self.divide_trapezoid_into_grid(box, (10, 10))
            for row in grid_centers1:
                for center1 in row:
                    # 绘制中心点
                    cv2.circle(self.cv_image, (int(center1[0][0]), int(center1[0][1])), radius=5, color=(255, 0, 255), thickness=-1)

            if detected:
                # 绘制检测到的棋盘
                cv_image = self.cv_image.copy()
                cv_image = cv2.drawContours(
                    cv_image, [box], -1, (0, 255, 0), 3)

                # 计算棋盘格宽高
                cell_width = width / board_size[0]
                cell_height = height / board_size[1]

                # 计算每个棋盘格的中心点
                grid_centers = []
                for i in range(board_size[1]-1):
                    for j in range(board_size[0]-1):
                        # 局部坐标（棋盘格中心点）
                        local_x = j * cell_width + cell_width 
                        local_y = i * cell_height + cell_height 

                        # 将偏移量应用到全局坐标系统中
                        global_x = local_x + center[0] - width/2
                        global_y = local_y + center[1] - height/2
                        
                        grid_centers.append((global_x, global_y))

                if angle<=45:
                    rotation_matrix = cv2.getRotationMatrix2D(center, -angle, 1)
                else:
                    rotation_matrix = cv2.getRotationMatrix2D(center, 90-angle, 1)

                rotated_centers = []
                for (x, y) in grid_centers:
                    # cv2.circle(cv_image, (int(x), int(y)),
                    #                5, (255, 0, 0), -1)
                    point = np.array([[x], [y], [1]])
                    rotated_point = np.dot(rotation_matrix, point)
                    rotated_centers.append(
                        (rotated_point[0][0], rotated_point[1][0]))
                    
                if request.getpositions:
                    for (x, y) in grid_centers:
                        # 转换坐标系
                        camera_xyz = self.getObject3DPosition(x, y)
                        camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 四舍五入到3位小数
                        world_position = self.tf_transform(camera_xyz)
                        positions.append(world_position)

                for (x_rot, y_rot) in rotated_centers:
                    cv2.circle(cv_image, (int(x_rot), int(y_rot)),
                                5, (255, 0, 0), -1)
                    cropped_image=self.crop_circles(cv_image,x_rot,y_rot,cell_width)
                    # cv2.imwrite(f"{x_rot}_{y_rot}.jpg", cropped_image)
                    if cropped_image.size > 0:
                        has_piece = self.check_for_piece(cropped_image)
                        if has_piece == 1 or has_piece == 0:
                            round += 1
                        board.append(has_piece)   

                # # 遍历每个棋盘格并裁剪
                # for i in range(board_size[1]-1):
                #     for j in range(board_size[0]-1):
                #         # 按存储顺序(行优先)访问
                #         cx, cy = rotated_centers[i * board_size[1]-1 + j]
                #         corners = self.get_rotated_rect_corners(
                #             (cx, cy), (cell_width, cell_height), angle)

                #         cropped_image = self.crop_polygon(cv_image, corners)
                #         # cv2.imwrite(f"{i}_{j}.jpg", cropped_image)

                #         if cropped_image.size > 0:
                #             has_piece = self.check_for_piece(cropped_image)
                #             if has_piece == 1 or has_piece == 0:
                #                 round += 1
                #             board.append(has_piece)

                try:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
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
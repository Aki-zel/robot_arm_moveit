#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_msgs.srv import (
    Get_Board_State,
    Get_Board_StateResponse,
    Hand_Catch,
    Hand_CatchResponse,
    Hand_CatchRequest,
)
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
            "color_detect", Hand_Catch, self.handle_color_detection
        )
        self.service2 = rospy.Service(
            "chessboard_detect", Get_Board_State, self.handle_chessboard_detection
        )
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
        cv2.imwrite(image_path, cv_image)  # 保存处理后的图像
        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])
        # 创建颜色掩码
        mask = cv2.inRange(hsv_image, lower_color, upper_color)
        image_path = os.path.join(current_work_dir, "2.png")
        cv2.imwrite(image_path, mask)  # 保存处理后的图像
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
            contours, key=lambda contour: cv2.boundingRect(contour)[0], reverse=True
        )

        # 将检测到的目标信息保存到列表中
        objects_info = []
        for contour in contours:
            area = cv2.contourArea(contour)
            # epsilon = 0.02 * cv2.arcLength(contour, True)
            # approx = cv2.approxPolyDP(contour, epsilon, True)
            print(area)  # 输出轮廓面积
            if self.area_threshold["min_area"] < area < self.area_threshold["max_area"]:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                # 计算物体中心点和角度
                center_x, center_y = rect[0]
                width, height = rect[1]
                angle = rect[2]
                # 根据检测框宽高调整角度，使得夹取间距较短
                if width <= height:  # 默认左下边为宽，顺时针转为正方向
                    angle = angle
                else:
                    angle = angle - 90
                # cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                cv2.circle(cv_image, (int(center_x), int(center_y)),
                           5, (0, 0, 255), -1)

                objects_info.append(
                    {
                        "label": color_name,
                        "center_x": int(center_x),
                        "center_y": int(center_y),
                        "angle": angle,
                    }
                )
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
                    label = obj["label"]
                    center_x = obj["center_x"]
                    center_y = obj["center_y"]
                    angle = obj["angle"]

                    camera_xyz = self.getObject3DPosition(center_x, center_y)
                    if camera_xyz == [0, 0, 0]:
                        continue
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()
                    world_position = self.tf_transform(camera_xyz)
                    pose = self.transform_pose(world_position, angle)
                    rospy.loginfo(pose.pose.position.z)
                    response.labels.append(label)
                    response.positions.append(pose)

            else:
                rospy.logwarn("No image received yet.")
        return response

    # 颜色检测
    def detect_color(self, hsv_image, color_name):
        # 获取颜色的阈值范围
        lower = np.array(self.colors[color_name]["color_threshold"]["lower"])
        upper = np.array(self.colors[color_name]["color_threshold"]["upper"])

        # 创建颜色掩码并进行形态学开运算
        mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                np.ones((5, 5), np.uint8))

        # 查找掩码中的轮廓
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if (
                self.colors[color_name]["area_threshold"]["min_area"]
                < area
                < self.colors[color_name]["area_threshold"]["max_area"]
            ):
                return True
        return False

    # 检测棋盘格内棋子
    def check_for_piece(self, cell_img):
        # 将输入图像转换为 HSV 色域
        hsv_image = cv2.cvtColor(cell_img, cv2.COLOR_BGR2HSV)

        # 检查己方
        player_detected = self.detect_color(hsv_image, "green")

        # 检查敌方
        opponent_detected = self.detect_color(hsv_image, "orange")
        if opponent_detected==False:
            opponent_detected = self.detect_color(hsv_image, "blue")
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
        points = np.array(
            [  # 未旋转时的四个顶点坐标
                [-w / 2, -h / 2],
                [w / 2, -h / 2],
                [w / 2, h / 2],
                [-w / 2, h / 2],
            ]
        )
        R = cv2.getRotationMatrix2D((0, 0), -angle, 1)  # 创建旋转矩阵
        points = (
            np.dot(points, R[:, :2].T) + center
        )  # 将顶点坐标转换为相对于中心点的坐标
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

    def crop_circles(self, image, x, y, diameter):
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
        cv2.circle(mask, (int(x), int(y)), radius,
                   (255, 255, 255), thickness=-1)
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

    # 获取最小包围四边形
    def get_min_enclosing_quadrilateral(self, hull):
        # 计算凸包
        epsilon = 0.01 * cv2.arcLength(hull, True)  # 设定逼近精度
        quadrilateral = cv2.approxPolyDP(hull, epsilon, True)  # 逼近多边形
        count = 0
        # 如果逼近结果不是四边形，调整epsilon
        while len(quadrilateral) != 4:
            count += 1
            epsilon += 0.01 * cv2.arcLength(hull, True)
            quadrilateral = cv2.approxPolyDP(hull, epsilon, True)
            if count == 5:
                return None

        return quadrilateral

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
            epsilon = 0.02 * cv2.arcLength(
                contour, True
            )  # 轮廓的近似程度，设为轮廓周长的1%,越小越近似
            approx = cv2.approxPolyDP(contour, epsilon, True)  # 轮廓的近似多边形
            area = cv2.contourArea(approx)  # 计算轮廓面积
            # 满足轮廓面积条件(顶点数量为4并且大于棋子的轮廓面积)，则将轮廓添加到列表中
            if len(approx) == 4 and cv2.isContourConvex(approx) and area > 1500:
                # 获取轮廓的最小外接矩形
                rect = cv2.minAreaRect(approx)
                width, height = rect[1]

                # 确保宽高非零，计算宽高比
                if width > 0 and height > 0:
                    aspect_ratio = max(width, height) / min(width, height)

                    # 限制宽高比 (例如：在1到1.5之间)
                    if 1.0 <= aspect_ratio <= 1.5:
                        squares.append(approx)

        if len(squares) > 0:
            # 计算每个四边形的中心点
            centers = []
            for square in squares:
                M = cv2.moments(square)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    centers.append((center_x, center_y))

            # 计算这些中心点的平均位置
            avg_center = np.mean(centers, axis=0)

            # 根据与平均位置的距离排除独立的四边形
            filtered_squares = []
            for i, center in enumerate(centers):
                distance = np.linalg.norm(np.array(center) - avg_center)
                if distance < 100:  # 设置距离阈值，排除偏离过远的四边形
                    filtered_squares.append(squares[i])

            if len(filtered_squares) > 0:
                # 合并轮廓并计算凸包
                merged_contour = np.vstack(filtered_squares).squeeze()
                hull = cv2.convexHull(merged_contour)  # 计算凸包,凸包是包含轮廓所有点的最小凸多边形
                min_enclosing_quadrilateral = self.get_min_enclosing_quadrilateral(
                    hull)  # 获取凸包的最小外接四边形
                # 计算最小外接矩形
                rect = cv2.minAreaRect(hull)
                center, (width, height), angle = rect
                center = np.array(center)
                return True, center, width, height, angle, min_enclosing_quadrilateral
            else:
                return False, None, None, None, None, None
        else:
            return False, None, None, None, None, None

    # 使用透视变换分割畸变棋盘格
    def divide_trapezoid_into_grid(self, hull, board_size=(10, 10)):
        intersection_points = []
        if len(hull) == 4:
            P1, P2, P3, P4 = hull[0][0], hull[1][0], hull[2][0], hull[3][0]
            # 按 y 坐标排序
            points = sorted([P1, P2, P3, P4], key=lambda p: p[1])
            # 找到上边的两个点（y 较小的点）
            top_points = points[:2]
            # 找到下边的两个点（y 较大的点）
            bottom_points = points[2:]
            # 左上角: 在上边的点中 x 最小的
            left_top = min(top_points, key=lambda p: p[0])
            # 右上角: 在上边的点中 x 最大的
            right_top = max(top_points, key=lambda p: p[0])
            # 左下角: 在下边的点中 x 最小的
            left_bottom = min(bottom_points, key=lambda p: p[0])
            # 右下角: 在下边的点中 x 最大的
            right_bottom = max(bottom_points, key=lambda p: p[0])

            # 获取透视变换矩阵
            rect = np.array([[0, 0], [board_size[0]-1, 0], [board_size[0]-1,
                            board_size[1]-1], [0, board_size[1]-1]], dtype="float32")
            # 构造dst数组构造变换后的点
            dst = np.array([left_top, right_top, right_bottom,
                           left_bottom], dtype="float32")
            # 透视变换矩阵
            M = cv2.getPerspectiveTransform(rect, dst)

            # 生成均匀分布的矩形网格点
            grid_points = []
            for i in range(board_size[0]):
                for j in range(board_size[1]):
                    grid_points.append((i, j))

            # # 将矩形网格点变换回图像中的梯形区域
            # for point in grid_points:
            #     point_transformed = cv2.perspectiveTransform(
            #         np.array([[point]], dtype="float32"), M)
            #     transformed_point = tuple(point_transformed[0][0])

            #     # 使用 pointPolygonTest 检查点是否在凸包内
            #     if cv2.pointPolygonTest(hull, transformed_point, False) >= 0:
            #         intersection_points.append(transformed_point)
            
            # 将矩形网格点变换回图像中的梯形区域
            for point in grid_points:
                point_transformed = cv2.perspectiveTransform(
                    np.array([[point]], dtype="float32"), M)
                intersection_points.append(tuple(point_transformed[0][0]))

        return intersection_points

    # 棋盘检测回调函数
    def handle_chessboard_detection(self, request):
        response = Get_Board_StateResponse()
        positions = []
        board = []
        round = 0
        board_size = (12, 10)

        if request.run:
            if self.cv_image is None:
                rospy.logwarn("尚未接收到图像。")
                response.board = []
                response.positions = []
                response.round = 0
                return response

            # 调用棋盘检测函数
            detected, center, width, height, angle, box = self.chessboard_detect(
                self.cv_image
            )

            if detected:
                # 绘制检测到的棋盘
                cv_image = self.cv_image.copy()
                grid_centers = self.divide_trapezoid_into_grid(
                    box, board_size)
                # # 计算棋盘格宽高
                cell_width = width / board_size[0]
                cell_height = height / board_size[1]

                # # 计算每个棋盘格的中心点
                # grid_centers = []
                # for i in range(board_size[1] - 1):
                #     for j in range(board_size[0] - 1):
                #         # 局部坐标（棋盘格中心点）
                #         local_x = j * cell_width + cell_width
                #         local_y = i * cell_height + cell_height

                #         # 将偏移量应用到全局坐标系统中
                #         global_x = local_x + center[0] - width / 2
                #         global_y = local_y + center[1] - height / 2

                #         grid_centers.append((global_x, global_y))

                # if angle <= 45:
                #     rotation_matrix = cv2.getRotationMatrix2D(
                #         center, -angle, 1)
                # else:
                #     rotation_matrix = cv2.getRotationMatrix2D(
                #         center, 90 - angle, 1)

                # rotated_centers = []
                # for x, y in grid_centers:
                #     # cv2.circle(cv_image, (int(x), int(y)),
                #     #                5, (255, 0, 0), -1)
                #     point = np.array([[x], [y], [1]])
                #     rotated_point = np.dot(rotation_matrix, point)
                #     rotated_centers.append(
                #         (rotated_point[0][0], rotated_point[1][0]))
                if request.getpositions:
                    i = 0
                    for x, y in grid_centers:
                        x_index = i // board_size[1]
                        y_index= i % board_size[1]
                        i += 1
                        if x_index==0 or x_index==board_size[0]-1 or y_index==0 or y_index==board_size[1]-1:
                            continue
                        # 转换坐标系
                        camera_xyz = self.getObject3DPosition(x, y)
                        camera_xyz = np.round(
                            np.array(camera_xyz), 3
                        ).tolist()  # 四舍五入到3位小数
                        world_position = self.tf_transform(camera_xyz)
                        positions.append(world_position)
                i=0
                for x_rot, y_rot in grid_centers:

                    x_index = i // board_size[1]
                    y_index= i % board_size[1]
                    i += 1
                    if x_index==0 or x_index==board_size[0]-1 or y_index==0 or y_index==board_size[1]-1:
                        continue
                    # cv2.circle(cv_image, (int(x_rot), int(y_rot)),
                    #            5, (255, 0, 255), -1)
                    # 在点的旁边绘制序号
                    # cv2.putText(cv_image, (int(x_rot) + 10, int(y_rot) + 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                    # cv2.imwrite(str(i)+".png",cv_image)
                    cropped_image = self.crop_circles(
                        cv_image, x_rot, y_rot, cell_width
                    )
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
                    cv_image = cv2.drawContours(
                        cv_image, [box], -1, (0, 255, 0), 3)
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


if __name__ == "__main__":
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

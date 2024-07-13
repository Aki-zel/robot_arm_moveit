import cv2
import numpy as np

cap = cv2.VideoCapture(6)  # 打开摄像头

while True:
    ret, frame = cap.read()  # 读取摄像头的一帧
    if not ret:
        print("Unable to receive frame (stream end?). Exiting ...")
        break

    # 将图像从BGR颜色空间转换为HSV颜色空间
    hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 定义柜子的HSV范围
    lower_cabinet = np.array([0, 0, 60])
    upper_cabinet = np.array([60, 60, 180])

    # 创建一个遮罩，范围内的颜色为柜子，其余部分为黑色
    cabinet_mask = cv2.inRange(hsv_image, lower_cabinet, upper_cabinet)

    # 使用遮罩在原图上提取柜子部分
    cabinet_areas = cv2.bitwise_and(frame, frame, mask=cabinet_mask)

    # 进行形态学开运算去除噪点
    kernel1 = np.ones((9, 9), np.uint8)
    morph_open = cv2.morphologyEx(cabinet_mask, cv2.MORPH_OPEN, kernel1)
    cv2.imshow('Morph Open', morph_open)
    # 添加闭运算操作以合并连通域
    kernel2 = np.ones((5, 5), np.uint8)
    morph_close = cv2.morphologyEx(morph_open, cv2.MORPH_CLOSE, kernel2)
    cv2.imshow('Morph Close', morph_close)

    # 查找柜子的轮廓
    contours, _ = cv2.findContours(morph_close, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 绘制柜子的边界框
    if contours:
        # 找到面积最大的柜子轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cabinet_roi = frame[y:y+h, x:x+w]  # 提取柜子区域

        # 将柜子区域转换为HSV色彩空间
        cabinet_hsv = cv2.cvtColor(cabinet_roi, cv2.COLOR_BGR2HSV)

        # 定义把手的HSV范围
        lower_handle = np.array([100, 30, 100])
        upper_handle = np.array([255, 255, 200])

        # 创建一个遮罩，范围内的颜色为把手，其余部分为黑色
        handle_mask = cv2.inRange(cabinet_hsv, lower_handle, upper_handle)

        # 使用遮罩在柜子区域内提取把手部分
        handle_areas = cv2.bitwise_and(cabinet_roi, cabinet_roi, mask=handle_mask)

        # 查找把手的轮廓并绘制边界框
        handle_contours, _ = cv2.findContours(handle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 滤除h>w的干扰轮廓
        handle_contours = [cnt for cnt in handle_contours if cv2.boundingRect(cnt)[2] >= cv2.boundingRect(cnt)[3]]

        for handle_contour in handle_contours:
            area = cv2.contourArea(handle_contour)
            if 1000 < area < 8000:
                hx, hy, hw, hh = cv2.boundingRect(handle_contour)
                cv2.rectangle(cabinet_roi, (hx, hy), (hx+hw, hy+hh), (255, 0, 0), 2)

    # 显示原始图像和检测结果
    cv2.imshow('Original Image', frame)
    cv2.imshow('Cabinet Areas', cabinet_areas)

    # 检查是否按下了 'q' 键来退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放摄像头资源并关闭所有窗口
cap.release()
cv2.destroyAllWindows()

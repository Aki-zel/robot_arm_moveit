import time
import freetype
import numpy as np
import cv2

# 加载字体
face = freetype.Face('/usr/share/fonts/truetype/华文仿宋.ttf')
face.set_pixel_sizes(0, 5) 
# 加载字符 'A'
face.load_char('好', freetype.FT_LOAD_NO_HINTING | freetype.FT_LOAD_NO_BITMAP)

# 获取字符的轮廓
outline = face.glyph.outline
bbox = outline.get_bbox()
print("Bounding Box:", bbox)

# 计算边界框的宽度和高度
width = bbox.xMax - bbox.xMin
height = bbox.yMax - bbox.yMin

# 创建一个白色背景的图像来绘制路径
img = np.ones((720, 1280, 3), dtype=np.uint8) * 255  # 白色背景
img_size = 720

# 用于存储路径点
path_points = []
all_path_points = []
contours = outline.contours

# 遍历每个轮廓段
start_index = 0  # 初始起始索引
for contour_end in contours:
    # 计算每个轮廓段的结束点
    for i, (point, tag) in enumerate(zip(outline.points[start_index:contour_end + 1], outline.tags[start_index:contour_end + 1])):
        x, y = point
        # 转换坐标：FreeType 中的坐标系原点在左上角，OpenCV 中原点在左下角
        y = 300 - y  # 翻转 Y 轴
        x = x - bbox.xMin + 50
        y = y + bbox.yMin
        path_points.append((int(x + (img_size - width) // 2), int(y + (img_size - height) // 2)))

    all_path_points.append(path_points.copy())
    path_points.clear()
    start_index = contour_end + 1

# 使用 cv2.line 连接路径点，并确保连接起点和终点
for path in all_path_points:
    for i in range(len(path) - 1):
        # 连接相邻的点
        cv2.line(img, path[i], path[i + 1], (0, 0, 0), 2)  # 画黑色线，线宽为2
    
    # 如果路径不为空，确保闭合，连接最后一个点和第一个点
    if len(path) > 1:
        cv2.line(img, path[-1], path[0], (0, 0, 0), 2)  # 画黑色线，线宽为2

# 绘制路径上的每个点
for path in all_path_points:
    for point in path:
        cv2.circle(img, tuple(point), 3, (0, 0, 255), -1)  # 用红色圆点标记路径上的每个点
        cv2.imshow('Font Outline1', img)
        cv2.waitKey(50)
        time.sleep(0.1)

# 显示图像
cv2.imshow('Font Outline', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

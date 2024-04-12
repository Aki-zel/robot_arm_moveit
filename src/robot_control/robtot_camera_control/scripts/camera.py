#! /usr/bin/env python
import time
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import os
import yaml
import numpy as np
import fastdeploy.vision as vision
import fastdeploy as fd
from arm_controller_code.srv import *
import threading
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs

label_msg = {
    0: 'AD钙奶',
    1: '旺仔牛奶',
    2: '王老吉',
    3: '青岛啤酒',
    4: '青岛啤酒',
    5: '雪碧',
    6: '七喜',
    7: '维他命',
    8: '东鹏',
    9: '百事可乐',
    10: '可口可乐',
    11: '红牛',
    12: '星巴克',
    13: '星巴克',
    14: '雪花',
    15: '雪花',
    16: '芬达',
    17: '美年达',
    18: '美年达',
    19: '美年达',
    20: '特伦苏',
    21: '魔方',
    22: '魔方',
    23: '雪花',
    24: '旺仔',
    25: '旺仔',
    26: 'RIO啤酒',
    27: 'RIO啤酒',
    28: 'RIO啤酒',
    29: 'RIO啤酒',
    30: '益达',
    31: '益达',
    32: '金典',
    33: '李子园',
    34: '特伦苏',
    35: '优酸乳',
    36: '早餐奶',
    37: '加多宝'
}


class Detection:
    def __init__(self, config, device, Booltrt):
        self.result = None
        self.model = None
        self.config = config
        self.setOption(device, Booltrt)
        self.loadModel()
        self.depth_img = None
        self.camera_info = None
        self.setflag = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.image_pub = rospy.Publisher(
            '/cycloid/image_topic', Image, queue_size=10)
        self.bridge = CvBridge()

    def loadModel(self):
        self.model = vision.detection.PPYOLOE(
            self.config['model_file'],
            self.config['params_file'],
            self.config['config_file'],
            runtime_option=self.option
        )

    def setOption(self, type, trt):
        self.option = fd.RuntimeOption()
        if type == "kunlunxin":
            self.option.use_kunlunxin()
        if type == "ascend":
            self.option.use_ascend()
        if type == "gpu":
            self.option.use_gpu()
        if type == "cpu":
            self.option.use_cpu()
        if trt:
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = "cache/model.trt"
        return self.option

    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result

    def getFilteredObjects(self, score_threshold):
        filtered_objects = []
        object_info = {}
        if self.result is not None:
            for i, score in enumerate(self.result.scores):
                if score >= score_threshold:
                    object_info = {
                        'label': label_msg[self.result.label_ids[i]],
                        'score': score,
                        'box_coordinates': self.result.boxes[i]
                    }
                    if object_info:
                        filtered_objects.append(object_info)
        return filtered_objects

    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=0.7
        )
        return vis_im

    def depthImageCallback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def cameraInfoCallback(self, msg):
        self.camera_info = msg

    def transfromTfposition(self, position):
        x, y, z = position
        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.frame_id="camera_link"
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        camera_point.pose.orientation.w=1

        try:
            # 使用tf2将机械臂摄像头坐标系转换到base_link坐标系
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link', rospy.Time(0), rospy.Duration(1))
            transformed_point = tf2_geometry_msgs.do_transform_pose(camera_point,transform)
            rospy.loginfo(
                "转化后的坐标 %s", transformed_point)
            if transformed_point is not None:
                return transformed_point.pose.position.x, transformed_point.pose.position.y, transformed_point.pose.position.z
            else:
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None

    def getObject3DPosition(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None

        # 获取相机固有参数
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        depth_value = self.depth_img[y, x]
        if depth_value == 0:
            depth_value = 490

        # 将像素坐标转换为三维点
        X = (x - cx) * depth_value / fx / 1000
        Y = (y - cy) * depth_value / fy / 1000
        Z = depth_value/1000

        return X, Y, Z

    def publish_image(self, cv_image):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "rgb8")
            self.image_pub.publish(ros_image)
        except Exception as e:
            print(e)


current_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(current_dir, 'config.yaml')
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)
rospy.init_node('detect_node', anonymous=True)
# 创建检测器对象
detect = Detection(config, "gpu", False)


def image_callback(msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # detect.Predicts(cv_image)

        def predict_thread(image):
            detect.Predicts(image)
            # detect.getFilteredObjects(score_threshold=0.7)

        # Create a thread for Predicts and start it with the current image
        predict_thread = threading.Thread(
            target=predict_thread, args=(cv_image,))
        predict_thread.start()

        # if detect.setflag == 2:
        cv_image = detect.visual(cv_image)
        # cv2.imshow("RGB Image", cv_image)
        detect.publish_image(cv_image)
        # cv2.waitKey(1)

    except Exception as e:
        print(e)


def depth_image_callback(msg):
    detect.depthImageCallback(msg)


def camera_info_callback(msg):
    detect.cameraInfoCallback(msg)


def handle_detection(request):
    msg = request.msg
    labels = []
    scores = []
    positions = []
    # msg = "detect"
    if msg == "detect":
        # 获取过滤后的物体数据
        # while 1:
        filtered_objects = detect.getFilteredObjects(score_threshold=0.7)
        #     if filtered_objects:
        #         break
        # 输出被识别物体的坐标信息和深度信息
        for obj in filtered_objects:
            label = obj['label']
            score = obj['score']
            box_coords = obj['box_coordinates']
            x_center = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
            y_center = int((box_coords[1] + box_coords[3]) / 2)-10  # 计算物体中心点y坐标
            # 获取物体的三维坐标
            object_position = detect.getObject3DPosition(x_center, y_center)
            if object_position is not None:
                X, Y, Z = detect.transfromTfposition(object_position)
                labels.append(label)
                scores.append(score)
                positions.extend([X, Y, Z])
                print(
                    f"检测到 - Label: {label}, Score: {score}, 3D坐标: ({X}, {Y}, {Z})")

    respond = detectmsgResponse()
    respond.labels = labels
    respond.scores = scores
    respond.positions = positions
    return respond


def main():
    sever = rospy.Service("/cycloid/detect", detectmsg, handle_detection)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',
                     Image, depth_image_callback)
    rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',
                     CameraInfo, camera_info_callback)
    rospy.spin()


if __name__ == '__main__':
    main()

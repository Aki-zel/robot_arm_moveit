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

        # 读取阈值配置
        self.threshold = config["threshold"]
        self.color_threshold = self.threshold["color_threshold"]
        self.area_threshold = self.threshold["area_threshold"]

        rospy.loginfo("ColorDetectServer initialized")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def color_thresholding(self, cv_image):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 读取配置文件中的颜色阈值
        lower_color = np.array(self.color_threshold["lower"])
        upper_color = np.array(self.color_threshold["upper"])

        # 应用颜色阈值
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects_info = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.area_threshold["min_area"] < area < self.area_threshold["max_area"]:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int0(box)

                cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)

                center_x, center_y = rect[0]
                angle = rect[2]

                color = "detected_color"  # 可以根据需要更改颜色标签

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

                tf_published = False
                for obj in objects_info:
                    label = obj['label']
                    center_x = obj['center_x']
                    center_y = obj['center_y']
                    angle = obj['angle']

                    camera_xyz = self.getObject3DPosition(
                        center_x, center_y)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()

                    world_position = self.tf_transform(camera_xyz)

                    pose = PoseStamped()
                    pose.header.stamp = rospy.Time.now()
                    pose.header.frame_id = "camera_color_optical_frame"

                    pose.pose.position.x = world_position.pose.position.x
                    pose.pose.position.y = world_position.pose.position.y
                    pose.pose.position.z = world_position.pose.position.z

                    initial_quaternion = [0, 1, 0, 0]
                    rotation_quaternion = quaternion_from_euler(
                        0, 0, np.deg2rad(angle))
                    final_quaternion = quaternion_multiply(
                        initial_quaternion, rotation_quaternion)

                    pose.pose.orientation.x = final_quaternion[0]
                    pose.pose.orientation.y = final_quaternion[1]
                    pose.pose.orientation.z = final_quaternion[2]
                    pose.pose.orientation.w = final_quaternion[3]

                    response.labels.append(label)
                    response.positions.append(pose)
                    if len(camera_xyz) == 3 and not tf_published:
                        self.tf_broad(world_position)
                        tf_published = True

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
    config_path = os.path.join(current_work_dir, "config", "config.yaml")
    with open(config_path, "r") as config_file:
        config = yaml.safe_load(config_file)

    try:
        rospy.init_node("color_detect_server")
        ColorDetectServer(config)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Interrupted")

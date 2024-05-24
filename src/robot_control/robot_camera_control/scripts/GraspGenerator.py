import os
import time

import matplotlib.pyplot as plt
import numpy as np
import torch
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from utils.device import get_device
from inference.post_process import post_process_output
from utils.data.camera_data import CameraData
from utils.dataset_processing.grasp import detect_grasps
from utils.visualisation.plot import plot_grasp
current_work_dir = os.path.dirname(__file__)
print(current_work_dir)

class GraspGenerator:
    def __init__(self, saved_model_path, visualize=False):
        self.saved_model_path = saved_model_path
        self.model = None
        self.device = None
        self.visualize = visualize

        self.cam_data = CameraData(include_depth=True, include_rgb=True)
        self.bridge = CvBridge()
        self.scale = 0.001 

        # ROS setup
        rospy.init_node('grasp_generator', anonymous=True)
        rospy.Subscriber('/camera/color/image_raw', Image, self.rgb_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)

        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        # # Load camera pose and depth scale (from running calibration)
        self.cam_pose = np.loadtxt('/home/ydt/rwm_moveit/src/robot_control/robot_camera_control/scripts/1.txt', delimiter=' ')

        if visualize:
            self.fig = plt.figure(figsize=(10, 10))
        else:
            self.fig = None

    def rgb_callback(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

    def camera_info_callback(self, data):
        self.camera_info = data

    def load_model(self):
        print('Loading model...')
        self.model = torch.load(self.saved_model_path)
        self.device = get_device(force_cpu=False)

    def generate(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            return

        rgb = self.rgb_image
        depth = self.depth_image
        depth1= np.asarray(depth, dtype=np.float32)
            # self.depth_image *= self.scale
        depth1= np.expand_dims(depth1, axis=2)
        # print(depth)
        x, depth_img, rgb_img = self.cam_data.get_data(rgb=rgb, depth=depth1)

        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.model.predict(xc)

        q_img, ang_img, width_img = post_process_output(pred['pos'], pred['cos'], pred['sin'], pred['width'])
        grasps = detect_grasps(q_img, ang_img, width_img, no_grasps=1)

        if len(grasps) != 0:
            # Get grasp position from model output
            pos_z = depth[grasps[0].center[0] + self.cam_data.top_left[0], grasps[0].center[1] + self.cam_data.top_left[1]] * self.scale 
            pos_x = np.multiply(grasps[0].center[1] + self.cam_data.top_left[1] - self.camera_info.K[2],
                                pos_z / self.camera_info.K[0])
            pos_y = np.multiply(grasps[0].center[0] + self.cam_data.top_left[0] - self.camera_info.K[5],
                                pos_z / self.camera_info.K[4])

            if pos_z == 0:
                return

            target = np.asarray([pos_x, pos_y, pos_z])
            target.shape = (3, 1)
            print('\ntarget: ', target)

            # Convert camera to robot coordinates
            camera2robot = self.cam_pose
            target_position = np.dot(camera2robot[0:3, 0:3], target) + camera2robot[0:3, 3:]
            target_position = target_position[0:3, 0]

            # Convert camera to robot angle
            angle = np.asarray([0, 0, grasps[0].angle])
            angle.shape = (3, 1)
            target_angle = np.dot(camera2robot[0:3, 0:3], angle)
            print('\ntarget_angle: ', target_angle)
            # Concatenate grasp pose with grasp angle
            grasp_pose = np.append(target_position, target_angle[2])

            print('\ngrasp_pose: ', grasp_pose)

            # np.save(self.grasp_pose, grasp_pose)
        
            if self.fig:
                plot_grasp(fig=self.fig, rgb_img=self.cam_data.get_rgb(rgb, False), grasps=grasps, save=False)

    def run(self):
        print("start")
        while not rospy.is_shutdown():
            self.generate()
        


if __name__ == '__main__':
    saved_model_path = '/home/ydt/rwm_moveit/src/robot_control/robot_camera_control/model/model_cornell_0.98'
    visualize = True  # Set to True if visualization is needed
    grasp_gen = GraspGenerator(saved_model_path, visualize)
    # grasp_gen.load_model()
    grasp_gen.run()

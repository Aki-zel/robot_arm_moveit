import os
import time

import cv2
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

current_work_dir = os.path.dirname(__file__)
print(current_work_dir)


class GraspGenerator:
    def __init__(self, saved_model_path, visualize=False):
        self.saved_model_path = saved_model_path
        self.model = None
        self.device = None
        self.visualize = visualize
        self.cam_data = CameraData(include_depth=True, include_rgb=True)
        self.scale = 0.001

    def load_model(self):
        print('Loading model...')
        self.model = torch.load(self.saved_model_path)
        self.device = get_device(force_cpu=False)

    def Predict(self, rgb_image, depth_image, camera_info, topbox, bottombox):
        if rgb_image is None or depth_image is None or camera_info is None:
            return
        self.cam_data.setTopBottom(topbox, bottombox)
        rgb = rgb_image
        depth = depth_image
        depth1 = np.asarray(depth, dtype=np.float32)
        # self.depth_image *= self.scale
        depth1 = np.expand_dims(depth1, axis=2)
        # print(depth)
        x, depth_img, rgb_img = self.cam_data.get_data(rgb=rgb, depth=depth1)

        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.model.predict(xc)

        q_img, ang_img, width_img = post_process_output(
            pred['pos'], pred['cos'], pred['sin'], pred['width'])
        grasps = detect_grasps(q_img, ang_img, width_img, no_grasps=1)

        if len(grasps) != 0:
            pos_x = np.multiply(
                grasps[0].center[1] + self.cam_data.top_left[1])
            pos_y = np.multiply(
                grasps[0].center[0] + self.cam_data.top_left[0])

            target = np.asarray([pos_x, pos_y])
            target.shape = (2, 1)
            print('\ntarget: ', target)
            # Convert camera to robot angle
            angle = np.asarray([0, 0, grasps[0].angle])
            grasp_pose = np.append(target, angle)

            print('\ngrasp_pose: ', grasp_pose)
            if self.visualize:
                for g in grasps:
                    color = (0, 255, 0)  # 绿色
                    thickness = 2
                    cv2.circle(rgb, (g.center[1] + self.cam_data.top_left[1],
                               g.center[0] + self.cam_data.top_left[0]), 3, color, thickness)
                    cv2.rectangle(rgb, (g.as_gr().point))

            cv2.imshow('Rectangle', rgb)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            return grasp_pose


if __name__ == '__main__':
    saved_model_path = 'weights/model_cornell_0.98'
    visualize = True
    grasp_gen = GraspGenerator(saved_model_path, visualize)
    grasp_gen.load_model()

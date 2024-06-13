#!/usr/bin/env python3

from pathlib import Path
import rospy

from vgn.detection import VGN, select_local_maxima
from vgn.rviz import Visualizer
import vgn.srv
from vgn.utils import *
import cv_bridge
import rospy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import std_srvs.srv

from robot_helpers.ros import tf
from robot_helpers.ros.conversions import *
from vgn.perception import UniformTSDFVolume
import vgn.srv
from vgn.utils import *


class GraspServer:
    def __init__(self):
        # 载入参数
        self.load_parameters()
        self.integrate = False
        # tf转换
        tf.init()
        # 用于发布扫描的点云数据
        self.init_topics()
        # 订阅消息处理话题
        self.advertise_services()
        # 图像转换
        self.cv_bridge = cv_bridge.CvBridge()
        # 在rviz里展示
        self.vis = Visualizer(self.base_frame_id )
        rospy.loginfo("TSDF server ready")
        rospy.loginfo("VGN server ready")

    def load_parameters(self):
        self.frame_id = rospy.get_param("~frame_id")
        self.base_frame_id = rospy.get_param("~base_frame_id")
        self.length = rospy.get_param("~length")
        self.resolution = rospy.get_param("~resolution")
        self.depth_scaling = rospy.get_param("~depth_scaling")
        self.cam_frame_id = rospy.get_param("~camera/frame_id")
        info_topic = rospy.get_param("~camera/info_topic")
        self.depth_topic = rospy.get_param("~camera/depth_topic")
        msg = rospy.wait_for_message(info_topic, CameraInfo)
        self.intrinsic = from_camera_info_msg(msg)
        self.vgn = VGN(Path(rospy.get_param("~model")))

    def advertise_services(self):
        rospy.Service("reset_map", std_srvs.srv.Empty, self.reset)
        rospy.Service("toggle_integration", std_srvs.srv.SetBool, self.toggle)
        rospy.Service("get_scene_cloud", vgn.srv.GetSceneCloud,
                      self.get_scene_cloud)
        rospy.Service("get_map_cloud", vgn.srv.GetMapCloud, self.get_map_cloud)
        rospy.Subscriber(self.depth_topic, Image, self.sensor_cb)
        # 创建抓取服务器
        rospy.Service("predict_grasps", vgn.srv.PredictGrasps,
                      self.predict_grasps)

    def reset(self, req):
        self.tsdf = UniformTSDFVolume(self.length, self.resolution)
        self.vis.clear()
        self.vis.roi(self.frame_id,self.length)
        return std_srvs.srv.EmptyResponse()

    def toggle(self, req):
        self.integrate = req.data
        return std_srvs.srv.SetBoolResponse(success=True)

    #  订阅深度图像并进行转换
    def sensor_cb(self, msg):
        if self.integrate:
            depth = (
                self.cv_bridge.imgmsg_to_cv2(msg).astype(np.float32) * self.depth_scaling )
            extrinsic = tf.lookup(
                self.cam_frame_id, self.frame_id, msg.header.stamp, rospy.Duration(0.1)
            )
            self.tsdf.integrate(depth, self.intrinsic, extrinsic)

    def get_scene_cloud(self, req):
        scene_cloud = self.tsdf.get_scene_cloud()
        points = np.asarray(scene_cloud.points)
        msg = to_cloud_msg(self.frame_id, points)
        self.scene_cloud_pub.publish(msg)
        res = vgn.srv.GetSceneCloudResponse()
        res.scene_cloud = msg
        return res

    def get_map_cloud(self, req):
        map_cloud = self.tsdf.get_map_cloud()
        points = np.asarray(map_cloud.points)
        distances = np.asarray(map_cloud.colors)[:, [0]]
        msg = to_cloud_msg(self.frame_id, points, distances=distances)
        self.map_cloud_pub.publish(msg)
        res = vgn.srv.GetMapCloudResponse()
        res.voxel_size = self.tsdf.voxel_size
        res.map_cloud = msg
        return res

    def predict_grasps(self, req):
        # Construct the input grid
        voxel_size = req.voxel_size
        points, distances = from_cloud_msg(req.map_cloud)
        # tsdf_grid = map_cloud_to_grid(voxel_size, points, distances)
        tsdf_grid=self.tsdf.get_grid()
        # Compute grasps
        out = self.vgn.predict(tsdf_grid)
        grasps, qualities = select_local_maxima(voxel_size, out, threshold=0.9)

        # Visualize detections
        self.vis.grasps(self.frame_id, grasps, qualities)
        
        # Construct the response message
        res = vgn.srv.PredictGraspsResponse()
        res.grasps = [to_grasp_config_msg(g, q)
                      for g, q in zip(grasps, qualities)]
        return res

    def init_topics(self):
        self.scene_cloud_pub = rospy.Publisher(
            "scene_cloud", PointCloud2, queue_size=1)
        self.map_cloud_pub = rospy.Publisher(
            "map_cloud", PointCloud2, queue_size=1)


if __name__ == "__main__":
    rospy.init_node("vgn_server")
    rospy.loginfo("start")
    GraspServer()
    rospy.spin()

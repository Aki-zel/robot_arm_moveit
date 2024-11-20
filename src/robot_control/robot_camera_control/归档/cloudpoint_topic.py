#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from robot_control.utils.preprocess import PreProcess
from sensor_msgs.msg import PointField


class PointCloudPreprocessor:
    def __init__(self):
        self.cloud_pub = rospy.Publisher('preprocessed_pointcloud', PointCloud2, queue_size=1)

        # 初始化ROS节点
        rospy.init_node('point_cloud_preprocessor')

    def pointcloud_callback(self, data):
        # 将ROS的点云消息转换成Open3D的点云数据
        cloud = o3d.geometry.PointCloud()
        points = []
        for p in point_cloud2.read_points(data):
            points.append([p[0], p[1], p[2]])
        cloud.points = o3d.utility.Vector3dVector(points)

        # 对点云进行预处理
        processed_cloud = PreProcess().preprocess(cloud)

        # 转换回ROS的点云消息格式
        ros_cloud_msg = self.convert_to_ros_cloud(processed_cloud)
        # 发布处理后的点云
        self.cloud_pub.publish(ros_cloud_msg)

    def convert_to_ros_cloud(self, processed_cloud):
        # 将Open3D点云转换回ROS PointCloud消息
        header = Header(stamp=rospy.Time.now(), frame_id="camera_color_optical_frame")
        ros_points = [(x, y, z) for x, y, z in processed_cloud.points]
        
        # 创建点云字段
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        # 创建 PointCloud2 消息
        ros_cloud_msg = point_cloud2.create_cloud(header, fields, ros_points)
        return ros_cloud_msg

# 实例化预处理器类
preprocessor = PointCloudPreprocessor()
# 订阅点云话题
rospy.Subscriber("/camera/depth/color/points", PointCloud2, preprocessor.pointcloud_callback)
# 循环等待接收点云数据
rospy.spin()

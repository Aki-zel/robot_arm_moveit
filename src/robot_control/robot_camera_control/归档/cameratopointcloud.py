#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np

class DepthToPointCloud:
    def __init__(self):
        self.pc_pub = rospy.Publisher("/camera_pointcloud", PointCloud2, queue_size=10)
        
        # Get camera info once
        camera_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]

        # Subscribe to depth image
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

    def depth_callback(self, data):
        try:
            # Convert depth image to numpy array
            depth_image = ros_numpy.numpify(data).astype(np.float32)

            # Downsample the depth image (e.g., by a factor of 2)
            depth_image = depth_image[::5, ::5]
            height, width = depth_image.shape
            timestamp = data.header.stamp
            points = []
            for v in range(height):
                for u in range(width):
                    z = depth_image[v, u] / 1000.0  # Convert depth from millimeters to meters
                    if z > 0 and z <= 1.0:
                        x = (u - self.cx / 5) * z / (self.fx / 5)
                        y = (v - self.cy / 5) * z / (self.fy / 5)
                        points.append((x, y, z))

            header = data.header
            header.stamp = timestamp 
            point_cloud = pc2.create_cloud_xyz32(header, points)
            self.pc_pub.publish(point_cloud)

        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

if __name__ == "__main__":
    rospy.init_node("depth_to_pointcloud", anonymous=True)
    DepthToPointCloud()
    rospy.spin()

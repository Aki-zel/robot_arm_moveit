import rospy
from cv_bridge import CvBridge
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Image, CameraInfo


class Transform:
    def __init__(self):
        self.bridge = CvBridge()

        # 订阅深度图像和相机参数
        self.depth_img = None
        self.camera_info = None
        rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_cb)
        rospy.Subscriber(
            '/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_cb)

        # 创建TF2缓存器和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 创建位置发布者
        self.object_position_pub = rospy.Publisher(
            "/object_position", PoseStamped, queue_size=10)

    def depth_image_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def camera_info_cb(self, msg):
        self.camera_info = msg

    def get_object_3d_position(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None

        # 获取相机固有参数
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        depth_value = self.depth_img[y, x]
        if depth_value == 0:
            return None

        # 将像素坐标转换为三维点
        x = (x - cx) * depth_value / fx / 1000
        y = (y - cy) * depth_value / fy / 1000
        z = depth_value / 1000

        return x, y, z

    def tf_transform(self, position):
        x, y, z = position
        camera_point = geometry_msgs.msg.PoseStamped()
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        camera_point.pose.orientation.w = 1

        try:
            # 使用tf2将机械臂摄像头坐标系转换到base_link坐标系
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1))
            world_point = tf2_geometry_msgs.do_transform_pose(
                camera_point, transform)
            if world_point is not None:
                rospy.loginfo("World point: %s", world_point)
                self.object_position_pub.publish(world_point)
                return world_point
            else:
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None

    # 发布物体的TF坐标系
    def tf_broad(self, position):
        x, y, z = position
        tfs = TransformStamped()  # 创建广播数据
        tfs.header.frame_id = "camera_color_optical_frame"  # 参考坐标系
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "object"  # 目标坐标系
        tfs.transform.translation.x = x
        tfs.transform.translation.y = y
        tfs.transform.translation.z = z
        tfs.transform.rotation.w = 1
        # 发布tf变换
        self.tf_broadcaster.sendTransform(tfs)

#!/usr/bin/env python

import sys
import cv2
import numpy as np
import rospy
import moveit_commander
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import tf.transformations as tf_trans

class HandEyeCalibration:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('hand_eye_calibration', anonymous=True)

        # 初始化MoveIt Commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("arm")  # 确保组名为你的机械臂控制组名

        # 初始化CvBridge
        self.bridge = CvBridge()

        # ROS订阅图像话题和相机内参话题
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.camera_info_callback)

        # 角点检测结果的发布者
        self.calib_result_pub = rospy.Publisher("/calibration/result", Bool, queue_size=10)

        # 订阅触发校准的标志位
        self.trigger_sub = rospy.Subscriber('/trigger_calibration', Bool, self.trigger_callback)

        # 初始化存储检测角点和机器人姿态的列表
        self.img_points = []
        self.robot_poses_rot = []  # 存储旋转矩阵
        self.robot_poses_trans = []  # 存储平移向量
        self.target_poses_rot = []  # 存储目标相对于相机的旋转矩阵
        self.target_poses_trans = []  # 存储目标相对于相机的平移向量

        # 初始化相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        self.cv_image = None

        # 棋盘格尺寸（内角点数）和正方形边长
        self.board_size = (11, 8)
        self.square_size = 0.015  # 正方形边长, 单位：米

        # 触发标志
        self.trigger_calibration = False

    def trigger_callback(self, msg):
        self.trigger_calibration = msg.data
        if self.trigger_calibration:
            rospy.loginfo("Calibration trigger received.")

    def camera_info_callback(self, data):
        if not self.camera_info_received:
            # 提取相机内参
            self.camera_matrix = np.array(data.K).reshape(3, 3)
            self.dist_coeffs = np.array(data.D)
            self.camera_info_received = True
            rospy.loginfo("Camera intrinsic parameters received.")

    def image_callback(self, data):
        if not self.camera_info_received:
            rospy.logwarn("Waiting for camera intrinsic parameters...")
            return
        # 将ROS图像消息转换为OpenCV格式
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

    def get_image_pose(self):
        while True:
            if self.cv_image is not None:

                # 转换为灰度图
                gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

                # 检测棋盘格角点
                ret, corners = cv2.findChessboardCorners(gray, self.board_size,flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                               cv2.CALIB_CB_NORMALIZE_IMAGE)

                if ret:
                    # 进一步精确角点位置
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

                    self.img_points.append(corners2)

                    # 若触发标志为True，执行以下操作
                    if self.trigger_calibration:
                        self.trigger_calibration = False  # 执行后重置触发标志
                        current_pose = self.get_robot_pose()

                        if current_pose is not None:
                            R_gripper2base, t_gripper2base = self.pose_to_transform(current_pose)
                            self.robot_poses_rot.append(R_gripper2base)
                            self.robot_poses_trans.append(t_gripper2base)
                              # 输出R_gripper2base和t_gripper2base
                            rospy.loginfo("R_gripper2base:")
                            rospy.loginfo(f"\n{R_gripper2base}")
                            rospy.loginfo("t_gripper2base:")
                            rospy.loginfo(f"\n{t_gripper2base}")
                            # 使用solvePnP求解目标相对于相机的位姿
                            obj_points = self.create_object_points()
                            ret, rvec, tvec = cv2.solvePnP(obj_points, corners2, self.camera_matrix, self.dist_coeffs)
                            R_target2cam, _ = cv2.Rodrigues(rvec)
                            rospy.loginfo("R_target2cam:")
                            rospy.loginfo(f"\n{R_target2cam}")
                            rospy.loginfo("tvec:")
                            rospy.loginfo(f"\n{tvec}")
                            self.target_poses_rot.append(R_target2cam)
                            self.target_poses_trans.append(tvec)
                                # 若达到足够的样本量，则进行标定
                            if len(self.robot_poses_rot) >= 10:
                                    self.calibrate_hand_eye()

                                            # 显示角点
                    cv2.drawChessboardCorners(self.cv_image, self.board_size, corners2, ret)


                cv2.imshow("Chessboard Corners", self.cv_image)
                cv2.waitKey(1)

    def get_robot_pose(self):
        try:
            # 获取机械臂末端当前姿态
            pose_stamped = self.group.get_current_pose()
            return pose_stamped.pose
        except Exception as e:
            rospy.logerr(f"Failed to get robot pose: {e}")
            return None

    def pose_to_transform(self, pose):
        # 将平移部分提取为向量
        tvec = np.array([pose.position.x, pose.position.y, pose.position.z])

        # 将四元数转换为旋转矩阵
        quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = tf_trans.quaternion_matrix(quaternion)[:3, :3]

        return rotation_matrix, tvec

    def create_object_points(self):
        # 创建棋盘格在世界坐标系中的3D点
        obj_points = np.zeros((np.prod(self.board_size), 3), dtype=np.float32)
        obj_points[:, :2] = np.indices(self.board_size).T.reshape(-1, 2)
        obj_points *= self.square_size
        return obj_points

    def calibrate_hand_eye(self):
        # 使用 cv2.calibrateHandEye 进行标定
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.robot_poses_rot,
            self.robot_poses_trans,
            self.target_poses_rot,
            self.target_poses_trans,
            method=cv2.CALIB_HAND_EYE_TSAI
        )

        # 打印并发布标定结果
        self.print_calibration_results(R_cam2gripper, t_cam2gripper)
        self.calib_result_pub.publish(True)

    def print_calibration_results(self, R_cam2gripper, t_cam2gripper):
        # 输出旋转矩阵
        rospy.loginfo("Camera to Gripper Rotation Matrix (R):")
        for row in R_cam2gripper:
            rospy.loginfo(f"{row}")
        # 输出平移向量
        rospy.loginfo("Camera to Gripper Translation Vector (t):")
        rospy.loginfo(f"[{t_cam2gripper[0][0]}, {t_cam2gripper[1][0]}, {t_cam2gripper[2][0]}]")
         # 将R_cam2gripper转换为4x4矩阵
        R_cam2gripper_4x4 = np.eye(4)
        R_cam2gripper_4x4[:3, :3] = R_cam2gripper

        # 将旋转矩阵转换为四元数
        quaternion = tf_trans.quaternion_from_matrix(R_cam2gripper_4x4)

        # 输出四元数
        rospy.loginfo("Camera to Gripper Quaternion (qx, qy, qz, qw):")
        rospy.loginfo(f"[{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")

if __name__ == '__main__':
    try:
        hand_eye_calib = HandEyeCalibration()
        hand_eye_calib.get_image_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

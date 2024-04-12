#!/usr/bin/env python3

# 导入依赖
import rospy
from std_msgs.msg import Bool
import actionlib
import fastdeploy as fd
import fastdeploy.vision as vision
import yaml
import pyrealsense2 as rs
import os
import time
import numpy as np
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import cv2
from robot_msgs.srv import *

pipeline = rs.pipeline()  # 定义流程pipeline
config = rs.config()  # 定义配置config
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # 流程开始
align_to = rs.stream.color  # 与color流对齐
align = rs.align(align_to)
current_work_dir = os.path.dirname(__file__)
running = False
actionLeable = [0,1,3]

def get_aligned_images():
    frames = pipeline.wait_for_frames()  # 等待获取图像帧
    aligned_frames = align.process(frames)  # 获取对齐帧
    aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
    color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

    ############### 相机参数的获取 #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）
    '''camera_parameters = {'fx': intr.fx, 'fy': intr.fy,
                         'ppx': intr.ppx, 'ppy': intr.ppy,
                         'height': intr.height, 'width': intr.width,
                         'depth_scale': profile.get_device().first_depth_sensor().get_depth_scale()
                         }'''

    # 保存内参到本地
    # with open('./intrinsics.json', 'w') as fp:
    #json.dump(camera_parameters, fp)
    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3通道深度图
    color_image = np.asanyarray(color_frame.get_data())  # RGB图

    # 返回相机内参、深度参数、彩色图、深度图、齐帧中的depth帧
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame
class Detection:
    def __init__(self, config):
        self.result = None
        self.model = None
        self.config = config
        self.setOption(self.config["device"])
        self.loadModel()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)


    def loadModel(self):
        self.model = vision.detection.YOLOv5(
            self.config['model_file'],
            self.config['params_file'],
            runtime_option=self.option
        )
    def setOption(self,type):
        self.option = fd.RuntimeOption()
        # if type == "gpu":
        #     self.option.use_gpu()
        if type == "cpu":
            self.option.use_cpu()
        if type == "openvino":
            self.option.use_cpu()
            self.option.use_openvino_backend()
        if type== "ort": 
            self.option.use_cpu()
            self.option.use_ort_backend()  
        if type== "trt":
            self.option.use_gpu()
            self.option.use_trt_backend()
            self.option.trt_option.serialize_file = current_work_dir+"/cache/model.trt"
        return self.option

    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result

    def getFilteredObjects(self):
        filtered=[]
        filtered_objects = []
        object_info = {}
        score_threshold=self.config["threshold"]["confidence"]
        iou=self.config["threshold"]["iou"]
        filtered=cv2.dnn.NMSBoxes(self.result.boxes,self.result.scores,score_threshold,iou)
        for i in filtered:
            object_info = {
                'label': self.config['class_name'][self.result.label_ids[i]],
                'score': self.result.scores[i],
                'box_coordinates': self.result.boxes[i]
            }
            # print (object_info)
            filtered_objects.append(object_info)
        return filtered_objects

    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )
        cv2.imwrite("/home/akria/rwm_moveit/src/robot_arm/arm_controller_code/scripts/img/"+str(time.time())+".jpg",vis_im)
        return vis_im
    
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
                return [transformed_point.pose.position.x, transformed_point.pose.position.y, transformed_point.pose.position.z]
            else:
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None
        
def getObjCoordinate(request):
    global model
    labels = []
    positions = []
    run=request.catch
    respond=Hand_CatchResponse()
    try:
        if run:
            for i in range(10):
                intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images() 
                cv2.waitKey(1)
                # rospy.Duration(0.01) # 获取对齐的图像与相机内参
            # cv2.imshow("img",color_image)
            t_start = time.time()  # 开始计时
            model.Predicts(color_image)
            t_end = time.time()  # 结束计时\
            model.visual(color_image)
            print("预测时间"+str((t_end-t_start)*1000))
            xyxy_list=model.getFilteredObjects()
            if xyxy_list:
                for obj in xyxy_list:
                    label = obj['label']
                    box_coords = obj['box_coordinates']
                    ux = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
                    uy = int((box_coords[1] + box_coords[3]) / 2)  # 计算物体中心点y坐标
                # 获取物体的三维坐标
                    print("[INFO] detect success")
                    dis = aligned_depth_frame.get_distance(ux, uy)
                    camera_xyz = rs.rs2_deproject_pixel_to_point(
                        depth_intrin, (ux, uy), dis)  # 计算相机坐标系的xyz
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 转成3位小数
                    # camera_xyz=model.transfromTfposition(camera_xyz)
                    if camera_xyz[2] < 100.0 and camera_xyz[2]!=0:
                        positions.extend(camera_xyz)
                        labels.append(label)
            respond.labels=labels
            respond.positions=positions
            print (respond)
            return respond
    except Exception as r:
        #print("[ERROR] %s"%r)
        rospy.loginfo("[ERROR] %s"%r)
        
if __name__ == '__main__':
    global model
    print("[INFO] wait for init")
    config_path=current_work_dir + '/config/yolov5s.yaml'
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    rospy.init_node("camrea_node")    
    model = Detection(config)
    service=rospy.Service("getcoordinate",Hand_Catch,getObjCoordinate)
    rospy.spin()
    cv2.destroyAllWindows()
    pipeline.stop()
    
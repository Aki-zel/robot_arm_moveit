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
import threading
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_geometry_msgs
from robot_msgs.srv import *
current_work_dir = os.path.dirname(__file__)
class yolo:
    def __init__(self, config):
        self.result = None
        self.model = None
        self.config = config
        self.setOption(self.config["device"])
        self.loadModel()
        self.cv_image=None
        self.depth_img = None
        self.camera_info = None
        self.setflag = 0
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster() # 创建TF广播

    def loadModel(self):
        if self.config['model_type']=="ppyoloe":
            self.model = vision.detection.PPYOLOE(
                    self.config['model_file'],
                    self.config['params_file'],
                    self.config['config_file'],
                    runtime_option=self.option
                    
            )
        elif self.config['model_type']=="yolov5":
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
    
    def depthImageCallback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)

    def cameraInfoCallback(self, msg):
        self.camera_info = msg

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")   
        except Exception as e:
            print(e)
    def visual(self, img):
        vis_im = vision.vis_detection(
            img, self.result, score_threshold=self.config["threshold"]["confidence"]
        )
        # cv2.imwrite("/home/akria/rwm_moveit/src/robot_arm/arm_controller_code/scripts/img/"+str(time.time())+".jpg",vis_im)
        return vis_im

    def getObject3DPosition(self, x, y):
        if self.depth_img is None or self.camera_info is None:
            return None

        # 获取相机固有参数
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        depth_value = self.depth_img[y, x]

        # 将像素坐标转换为三维点
        X = (x - cx) * depth_value / fx / 1000
        Y = (y - cy) * depth_value / fy / 1000
        Z = depth_value/1000

        return [X, Y, Z]
    def tf_transform(self, position):
        x, y, z = position
        camera_point = PoseStamped()
        camera_point.header.frame_id = "camera_color_optical_frame"
        camera_point.pose.position.x = x
        camera_point.pose.position.y = y
        camera_point.pose.position.z = z
        camera_point.pose.orientation.w = 1

        try:
            # 使用tf2将机械臂摄像头坐标系转换到base_link坐标系
            transform = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time(0), rospy.Duration(1))
            world_point = tf2_geometry_msgs.do_transform_pose(camera_point, transform)
            if world_point is not None:
                rospy.loginfo("World point: %s", world_point)
                self.object_position_pub.publish(world_point)
                self.tf_broad(world_point)
            else:
                return None
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Exception while transforming: %s", e)
            return None    
    def Predicts(self, img):
        self.result = self.model.predict(img)
        return self.result
    def tf_broad(self, position): 
        tfs = TransformStamped() # 创建广播数据
        tfs.header.frame_id = "base_link"  # 参考坐标系
        tfs.header.stamp = rospy.Time.now()
        tfs.child_frame_id = "object"  # 目标坐标系
        tfs.transform.translation = position.pose.position
        tfs.transform.rotation= position.pose.orientation
        # 发布tf变换
        self.tf_broadcaster.sendTransform(tfs)
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
    
    
def getObjCoordinate(request):
    global model
    labels = []
    positions = []
    run=request.catch
    respond=Hand_CatchResponse()
    try:
        if run:
            color_image=model.cv_image

            t_start = time.time()  # 开始计时
            model.Predicts(color_image)
            t_end = time.time()  # 结束计时\
            img=model.visual(color_image)
            # cv2.imshow("img",img)
            print("预测时间"+str((t_end-t_start)*1000))
            object_list=model.getFilteredObjects()
            if object_list:
                for obj in object_list:
                    label = obj['label']
                    box_coords = obj['box_coordinates']
                    ux = int((box_coords[0] + box_coords[2]) / 2)  # 计算物体中心点x坐标
                    uy = int((box_coords[1] + box_coords[3]) / 2)  # 计算物体中心点y坐标
                # 获取物体的三维坐标
                    print("[INFO] detect success")
                    camera_xyz=model.getObject3DPosition(ux,uy)
                    camera_xyz = np.round(np.array(camera_xyz), 3).tolist()  # 转成3位小数
                    if camera_xyz[2] < 100.0 and camera_xyz[2]!=0:
                        camera_xyz=model.tf_transform(camera_xyz)
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
    config_path=current_work_dir + '/config/yolov5s.yaml'
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    rospy.init_node("camrea_node")    
    model = yolo(config)
    rospy.Subscriber('/camera/color/image_raw', Image, model.image_callback)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',
                     Image, model.depthImageCallback)
    rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',
                     CameraInfo, model.cameraInfoCallback)
    service=rospy.Service("objection_detect",Hand_Catch,getObjCoordinate)
    rospy.spin()
    cv2.destroyAllWindows()
    
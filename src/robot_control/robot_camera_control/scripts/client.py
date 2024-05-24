#!/usr/bin/env python3
from robot_msgs.srv import *
import sys
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def add_two_ints_client(run):
    rospy.wait_for_service('objection_detect')
    try:
        # 创建服务代理
        detect_service = rospy.ServiceProxy('objection_detect', Hand_Catch)
        # 创建请求对象并设置属性
        request = Hand_CatchRequest()
        request.run = run
        # 调用服务并获取响应
        resp = detect_service(request)
        
        # 显示图像
        if resp.detect_image:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(resp.detect_image, desired_encoding="bgr8")
            cv2.imshow("Detection Image", cv_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        print (resp.labels, resp.positions)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def usage():
    return "%s [run]" % sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        run = sys.argv[1].lower() == 'true'
    else:
        print(usage())
        sys.exit(1)

    add_two_ints_client(run)

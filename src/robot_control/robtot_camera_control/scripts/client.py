#!/usr/bin/env python3
from robot_msgs.srv import *
import sys
import rospy

def add_two_ints_client(run):
    rospy.wait_for_service('objection_detect')
    try:
        # 创建服务代理
        detect_service = rospy.ServiceProxy('objection_detect', Hand_Catch)
        # 创建请求对象并设置属性
        request = Hand_CatchRequest()
        request.catch = run
        # 调用服务并获取响应
        resp1 = detect_service(request)
        print(resp1)
        return resp1
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

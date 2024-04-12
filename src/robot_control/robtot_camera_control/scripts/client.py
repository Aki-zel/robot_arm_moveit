#!/usr/bin/env python
from mwrobot_msgs.srv import *
import sys
import rospy
 
def add_two_ints_client(x):
    rospy.wait_for_service('getcoordinate')
    try:
        add_two_ints = rospy.ServiceProxy('getcoordinate', armgradm)
        resp1 = add_two_ints(x)
        print(resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
 
def usage():
    return "%s [x y]"%sys.argv[0]
 
if __name__ == "__main__":

    add_two_ints_client(True)
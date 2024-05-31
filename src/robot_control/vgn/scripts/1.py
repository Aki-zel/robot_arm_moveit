import geometry_msgs.msg
import numpy as np
import rospy
from std_srvs.srv import SetBool, Empty

rospy.init_node("111")

reset_map = rospy.ServiceProxy("reset_map", Empty)
toggle_integration = rospy.ServiceProxy("toggle_integration", SetBool)
reset_map()
toggle_integration(True)

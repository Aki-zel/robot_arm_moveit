#! /usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose

absolute_path = "/home/yds/rmrobot_ws/src/rm65_grasping/scripts"
sys.path.insert(0, absolute_path)


class GraspingDemo:
    def __init__(self):
         # 初始化ros节点
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('rm65_grasping', anonymous=False)

        self.armgroup = MoveGroupCommander('arm')
        #self.grippergroup = MoveGroupCommander('gripper')
        self.armgroup.set_goal_joint_tolerance(0.001)
        self.armgroup.set_goal_position_tolerance(0.001)
        self.armgroup.set_goal_orientation_tolerance(0.01)

    def go_image_pose(self):
		# 设定初始位姿
        joint_configuration = [0.108, -0.376, 1.37, 0.017, 1.371, 0.182]
        self.armgroup.set_joint_value_target(joint_configuration)
        self.armgroup.go()
        print("Point image_pose")
        rospy.sleep(1)

    def detect(self):
        pose = rospy.wait_for_message("/objection_position_pose", Pose)
        object_position_info = pose.position
        print(object_position_info)

        #保持末端位姿
        pose_target = self.armgroup.get_current_pose().pose
        pose_target.position.x = -0.334
        pose_target.position.y = -0.036
        pose_target.position.z = 0.4
        # 设置目标位姿
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()
        print("Point Home")
        rospy.sleep(1)

        # 移动到目标物体上方
        pose_target = self.armgroup.get_current_pose().pose
        pose_target.position.x = object_position_info.x+0.1
        pose_target.position.y = object_position_info.y
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()
        print("Point move")
        # 打开夹爪
        robot.Set_Tool_DO_State(num=2, state=False)

        # 靠近物体准备夹取
        pose_target = self.armgroup.get_current_pose().pose
        pose_target.position.z = object_position_info.z + 0.12
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()
        print("Point Catch")
        rospy.sleep(1)

        #夹取
        robot.Set_Tool_DO_State(num=2, state=True)

        # 抬起目标物体
        print("Point Lift")
        #rospy.sleep(1)
        rospy.sleep(1)
        pose_target.position.z += 0.1
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()

        # 放置目标物体
        print("Point Drop")
        rospy.sleep(1)
        pose_target.position.x = -0.334
        pose_target.position.y = 0.06
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()
        rospy.sleep(1)

        pose_target.position.z -= 0.1
        self.armgroup.set_pose_target(pose_target)
        self.armgroup.go()
        robot.Set_Tool_DO_State(num=2, state=False)

if __name__ == "__main__":
    #初始化机器人
    robot = Arm(65, "192.168.3.61")
    # 初始化夹爪
    robot.Set_Tool_DO_State(num=1, state=True)
    robot.Set_Tool_DO_State(num=1, state=False)
    robot.Set_Tool_DO_State(num=2, state=False)

    demo = GraspingDemo()
    demo.go_image_pose()
    demo.detect()
    demo.go_image_pose()
    
    robot.RM_API_UnInit()
    robot.Arm_Socket_Close()

    while not rospy.is_shutdown():
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

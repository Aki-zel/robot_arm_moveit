
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>
#include <MoveitServer.h>
#include <robot_msgs/Hand_Catch.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
using namespace std;


int main(int argc, char **argv)
{
	// 设置编码
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "moveit_control_server_cpp");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	std::string PLANNING_GROUP = "arm";
	MoveitServer arm(PLANNING_GROUP);
	ros::ServiceClient client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect"); // 创建目标检测服务客户端

	arm.move_j(std::vector<double>{arm.degreesToRadians(0), arm.degreesToRadians(-30), arm.degreesToRadians(-90),
											 arm.degreesToRadians(0), arm.degreesToRadians(-60), arm.degreesToRadians(-180)});
	
	// 调用目标检测服务
	robot_msgs::Hand_Catch srv;
	srv.request.run = true; // 设置请求标志位
	srv.request.color_name = "blue";
	if (client.call(srv))
	{
		ROS_INFO("Service call succeeded");

		const robot_msgs::Hand_CatchResponse &response = srv.response;
		std::map<std::string, geometry_msgs::PoseStamped> positions;
		for (size_t i = 0; i < response.labels.size(); ++i)
		{
			std::string label = response.labels[i];
			positions[label] = response.positions[i];
		}
		double stack_height = 0.0;
		// 定义堆叠位置（假设是一个固定位置）
		std::vector<double> stack_position;
		stack_position = {0.3, 0.0, 0.0, 0.0, 1, 0.0, 0.0};

		for (size_t i = 0; i < response.labels.size(); ++i)
		{
			std::string label = response.labels[i];
			if (positions.find(label) != positions.end())
			{
				geometry_msgs::PoseStamped p = positions[label];

				// 移动到目标上方
				p.pose.position.z += 0.10;
				arm.move_p(p);
				ROS_INFO("移动到目标上方");

				// 打开夹爪
				arm.Set_Tool_DO(2, false);
				ROS_INFO("夹爪开");

				// 移动到抓取位置
				p.pose.position.z -= 0.10;
				arm.move_p(p);
				ROS_INFO("夹取目标");

				// 关闭夹爪
				arm.Set_Tool_DO(2, true);

				// 移动到堆叠位置上方
				stack_position[2] = p.pose.position.z + 0.1 + stack_height;
				arm.move_p(stack_position);
				ROS_INFO("移动到堆叠位置上方");

				// 移动到堆叠位置
				stack_position[2] -= 0.1;
				arm.move_p(stack_position);
				ROS_INFO("放置目标");

				// 打开夹爪
				arm.Set_Tool_DO(2, false);
				ROS_INFO("夹爪开");

				// 更新堆叠高度
				stack_height += 0.07; // 根据方块的高度调整

				// 移动到安全位置
				vector<double> joints = {0, 0, arm.degreesToRadians(-90), 0, arm.degreesToRadians(-90), arm.degreesToRadians(180)};
				arm.move_j(joints);
				ROS_INFO("移动到安全位置");
			}
			else
			{
				ROS_ERROR("Failed to call service objection_detect");
			}
		}
	}

	ros::waitForShutdown();
	return 0;
}

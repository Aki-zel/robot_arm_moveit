
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

using namespace std;

int main(int argc, char **argv)
{
	// 设置编码
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "moveit_control_server_cpp");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	std::string PLANNING_GROUP = "arm";
	// moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	MoveitServer moveit_server(PLANNING_GROUP);
	ros::ServiceClient client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect"); // 创建目标检测服务客户端

	// Test

	// test for move_j
	cout << "-----------------------test for move_j----------------------" << endl;
	vector<double> joints = {0, 0, moveit_server.degreesToRadians(-90), 0, moveit_server.degreesToRadians(-90), moveit_server.degreesToRadians(180)};
	moveit_server.move_j(joints);
	ros::Duration(5.0).sleep();

	// test for move_p and move_l(1 point)
	// cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
	// vector<double> xyzrpy={0.31,0.067,0.11,0,1,0,0};
	// moveit_server.move_p(xyzrpy);

	// xyzrpy[2]=0.2;
	// moveit_server.move_l(xyzrpy);

	// // test for move_l (>=2 points)
	// cout<<"-----------------------test for move_l(more points)----------------------"<<endl;
	// vector<vector<double>> xyzrpys;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[1]=0.2;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[0]=0.4;
	// moveit_server.move_l(xyzrpys);

	// // test for my functions
	// cout<<"-----------------------test for my move_function----------------------"<<endl;
	// double xyz[3]={-0.5,0.02,0.2};
	// moveit_server.move_l(xyz);
	// ros::Duration(5.0).sleep();
	// xyz[2]=0.3;
	// moveit_server.move_l(xyz);
	// ros::Duration(5.0).sleep();
	// vector<double> xyzrpy={0.3,0.1,0.4,-3.1415,0,0};
	// vector<double> xyzrpy1={0.3,0.2,0.3,-3.1415,0,0};
	// moveit_server.move_l({xyzrpy, xyzrpy1});
	// xyz[2]=0.1;
	// moveit_server.move_p_with_constrains(xyz);

	// 控制夹爪
	// moveit_server.Set_Tool_DO(2, true);
	// ros::Duration(2.0).sleep();
	// moveit_server.Set_Tool_DO(2, false);
	// ros::Duration(2.0).sleep();
	// moveit_server.Set_Tool_DO(2, true);

	// 调用目标检测服务
	robot_msgs::Hand_Catch srv;
	srv.request.run = true; // 设置请求标志位
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
				moveit_server.move_p(p);
				ROS_INFO("移动到目标上方");

				// 打开夹爪
				moveit_server.Set_Tool_DO(2, false);
				ROS_INFO("夹爪开");

				// 移动到抓取位置
				p.pose.position.z -= 0.10;
				moveit_server.move_p(p);
				ROS_INFO("摘取目标");

				// 关闭夹爪
				moveit_server.Set_Tool_DO(2, true);				

				// 移动到堆叠位置上方
				stack_position[2] = p.pose.position.z + 0.1+ stack_height;
				moveit_server.move_p(stack_position);
				ROS_INFO("移动到堆叠位置上方");

				// 移动到堆叠位置
				stack_position[2] -= 0.1;
				moveit_server.move_p(stack_position);
				ROS_INFO("放置目标");

				// 打开夹爪
				moveit_server.Set_Tool_DO(2, false);
				ROS_INFO("夹爪开");

				// 更新堆叠高度
				stack_height += 0.07; // 根据方块的高度调整

				// 移动到安全位置
				vector<double> joints = {0, 0, moveit_server.degreesToRadians(-90), 0, moveit_server.degreesToRadians(-90), moveit_server.degreesToRadians(180)};
				moveit_server.move_j(joints);
				ROS_INFO("移动到安全位置");

				// for (size_t i = 0; i < response.labels.size(); ++i)
				// {
				// 	std::string label = response.labels[i];
				//     if (label == "red") {
				// 		geometry_msgs::PoseStamped p = response.positions[i];
				// 		// std::cout << "Position:" << std::endl;
				// 		// std::cout << "x: " << p.pose.position.x << std::endl;
				// 		// std::cout << "y: " << p.pose.position.y << std::endl;
				// 		// std::cout << "z: " << p.pose.position.z << std::endl;
				// 		// std::cout << "Orientation:" << std::endl;
				// 		// std::cout << "x: " << p.pose.orientation.x << std::endl;
				// 		// std::cout << "y: " << p.pose.orientation.y << std::endl;
				// 		// std::cout << "z: " << p.pose.orientation.z << std::endl;
				// 		// std::cout << "w: " << p.pose.orientation.w << std::endl;

				// 		// 移动到目标下方
				// 		p.pose.position.z += 0.10;
				// 		moveit_server.move_p(p);
				// 		ROS_INFO("移动到目标上方");
				// 		ros::Duration(2.0).sleep();
				// 		// 打开夹爪
				// 		moveit_server.Set_Tool_DO(2, false);
				// 		ROS_INFO("夹爪开");
				// 		ros::Duration(1.0).sleep();
				// 		// 移动到抓取位置
				// 		p.pose.position.z -= 0.10;
				// 		moveit_server.move_p(p);
				// 		ROS_INFO("摘取目标");
				// 		ros::Duration(1.0).sleep();
				// 		// 关闭夹爪
				// 		moveit_server.Set_Tool_DO(2, true);
				// 		ros::Duration(2.0).sleep();

				// 		vector<double> joints = {0, 0, moveit_server.degreesToRadians(-90), 0, moveit_server.degreesToRadians(-90), moveit_server.degreesToRadians(180)};
				// 		moveit_server.move_j(joints);

				// 		// 打开夹爪
				// 		moveit_server.Set_Tool_DO(2, false);
				// 		ROS_INFO("夹爪开");
				// 		ros::Duration(1.0).sleep();
			}
		}
		// 显示检测结果图像
		cv::Mat detect_image = cv_bridge::toCvCopy(response.detect_image, sensor_msgs::image_encodings::BGR8)->image;
		cv::imshow("Detection Results", detect_image);
		cv::waitKey(0); // 按下任意键继续
		cv::destroyAllWindows();
	}
	else
	{
		ROS_ERROR("Failed to call service objection_detect");
	}

	ros::waitForShutdown();
	return 0;
}


#include <ros/ros.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <bits/stdc++.h>
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
	MoveitServer arm(PLANNING_GROUP);
	robotTool tool;
	ros::waitForShutdown();
	return 0;
}

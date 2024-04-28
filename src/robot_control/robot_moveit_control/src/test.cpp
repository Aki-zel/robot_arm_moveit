
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

	// Test

	// test for move_j
	cout << "-----------------------test for move_j----------------------" << endl;
	vector<double> joints = {0, 0, 1.57, 0, 0, 0};
	moveit_server.move_j(joints);
	vector<double> joints1 = {0, 0, 0, 0, 0, 0};
	moveit_server.move_j(joints1);
	// // test for move_p and move_l(1 point)
	// cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
	// vector<double> xyzrpy={0.3,0.1,0.4,-3.1415,0,0};
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

	// // test for move_p_with constrains
	// cout<<"-----------------------test for move_p_with_constrains----------------------"<<endl;
	// vector<double> pose1={0.4,0,0.4,0,3.141592/2,0};
	// moveit_server.move_p(pose1);
	// vector<double> pose2={0.4,0.2,0.2,0,3.141592/2,0};
	// moveit_server.move_p_with_constrains(pose2);
	// vector<double> pose3={0.0,0.5,0.3,0,3.141592/2,0};
	// moveit_server.move_p_with_constrains(pose3);

	// // test for my functions
	// cout<<"-----------------------test for my move_function----------------------"<<endl;
	// double xyz[3]={0.3,0.1,0.4};
	// moveit_server.move_p(xyz);
	// xyz[2]=0.2;
	// moveit_server.move_l(xyz);
	// xyz[1]=0.2;
	// moveit_server.move_p_with_constrains(xyz);

	// 控制夹爪
	// moveit_server.Set_Tool_DO(2, true);
	// ros::Duration(2.0).sleep();
	// moveit_server.Set_Tool_DO(2, false);
	// ros::Duration(2.0).sleep();
	// moveit_server.Set_Tool_DO(2, true);
	ros::waitForShutdown();
	return 0;
}

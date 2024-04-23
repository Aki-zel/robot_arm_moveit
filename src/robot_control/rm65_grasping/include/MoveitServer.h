#ifndef MOVEITSERVER_H_
#define MOVEITSERVER_H_

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
#include <Tool_Digital_Output.h>
#include <Tool_IO_State.h>


using namespace std;


class MoveitServer
{
public:
	MoveitServer(const ros::NodeHandle &nh, moveit::planning_interface::MoveGroupInterface &arm, const string &PLANNING_GROUP);
	void go_home();
	bool move_j(const vector<double> &joint_group_positions);
	bool move_p(const vector<double> &pose);
	bool move_p(const double (&position)[3]);
	bool move_p_with_constrains(const vector<double>& pose);
	bool move_p_with_constrains(const double (&position)[3]);
	bool move_l(const vector<double>& pose);
	bool move_l(const double (&position)[3]);
	bool move_l(const vector<vector<double>>& posees);
	void Set_Tool_DO(int num, bool state);
	~MoveitServer();

public:
	string reference_frame;
	string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface *arm_;
	//msg
	ros::Publisher tool_do_pub;
};

#endif /* MOVEITSERVER_H_ */

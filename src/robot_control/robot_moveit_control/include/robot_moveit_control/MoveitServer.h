#ifndef MOVEITSERVER_H_
#define MOVEITSERVER_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <rm_msgs/Tool_IO_State.h>
#include <bits/stdc++.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// using namespace std;

class MoveitServer
{
public:
	MoveitServer(std::string &PLANNING_GROUP);
	void go_home();
	void go_pose(const std::string str);
	bool move_j(const std::vector<double> &joint_group_positions);
	bool move_p(const std::vector<double> &pose);
	bool move_p(const double (&position)[3]);
	bool move_p(const geometry_msgs::PoseStampedConstPtr &msg);
	bool move_p_with_constrains(const std::vector<double> &pose);
	bool move_p_with_constrains(const double (&position)[3]);
	bool move_l(const std::vector<double> &pose);
	bool move_l(const double (&position)[3]);
	bool move_l(const std::vector<std::vector<double>> &posees);
	void Set_Tool_DO(int num, bool state);
	void joint_state_callback(const sensor_msgs::JointStateConstPtr &msg);
	void Planer(moveit::planning_interface::MoveGroupInterface::Plan plan);
	bool Executer();
	~MoveitServer();

public:
	std::string reference_frame;
	std::string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface arm_;
	moveit::planning_interface::MoveGroupInterface::Plan myplan;
	// msg
	ros::Publisher tool_do_pub;
	ros::Subscriber joint_state_sub;
	sensor_msgs::JointState *joint_state;
};

#endif /* MOVEITSERVER_H_ */

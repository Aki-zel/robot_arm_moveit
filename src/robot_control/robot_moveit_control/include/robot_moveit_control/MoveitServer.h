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
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

typedef actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> MoveGroupClient;

class MoveitServer
{
public:
	MoveitServer(std::string &PLANNING_GROUP);
	void go_home();
	void go_pose(const std::string str);
	void move_j(const std::vector<double> &joint_group_positions, bool isAsync);
	void move_p(const std::vector<double> &pose, bool isAsync);
	void move_p(const geometry_msgs::Pose &msg, bool isAsync);
	void move_p_with_constrains(const std::vector<double> &pose);
	void move_p_with_constrains(const double (&position)[3]);
	void move_l(const std::vector<double> &pose);
	void move_l(const double (&position)[3]);
	void move_l(const std::vector<std::vector<double>> &posees);
	void Set_Tool_DO(int num, bool state);
	void tf_callback(const tf2_msgs::TFMessageConstPtr &transformStamped);
	geometry_msgs::Transform getCurrent_State();
	bool Planer();
	bool asyncPlaner();
	double round(double num, int exponent);
	geometry_msgs::Pose setPoint(const double x, const double y, const double z);
	geometry_msgs::Pose setPoint(const std::vector<double> &pose);
	~MoveitServer();
	void stop();
private:
	std::string reference_frame;
	std::string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface arm_;
	moveit::planning_interface::MoveGroupInterface::Plan isPlan;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener *tfListener;
	// msg
	ros::Subscriber tf_sub;
	geometry_msgs::Transform current_state;
	std::future<bool> last_task_future;
};

#endif /* MOVEITSERVER_H_ */

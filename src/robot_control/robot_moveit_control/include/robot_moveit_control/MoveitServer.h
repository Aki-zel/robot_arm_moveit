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

class TaskExecutor
{
public:
	TaskExecutor() : stop_requested_(false) {}

	void start()
	{
		worker_thread_ = std::thread(&TaskExecutor::worker, this);
	}

	void stop()
	{
		{
			std::unique_lock<std::mutex> lock(queue_mutex_);
			stop_requested_ = true;
		}
		condition_.notify_one(); // 通知工作线程退出
		worker_thread_.join();	 // 等待工作线程退出
	}

	void addTask(std::function<void()> task)
	{
		{
			std::unique_lock<std::mutex> lock(queue_mutex_);
			tasks_.push(task);
		}
		condition_.notify_one(); // 通知工作线程有新任务
	}

private:
	void worker()
	{
		while (true)
		{
			std::function<void()> task;
			{
				std::unique_lock<std::mutex> lock(queue_mutex_);
				condition_.wait(lock, [this]()
								{ return !tasks_.empty() || stop_requested_; });
				if (stop_requested_)
				{
					break; // 收到停止请求，退出线程
				}
				task = tasks_.front();
				tasks_.pop();
			}
			task(); // 执行任务
		}
	}

	std::thread worker_thread_;
	std::queue<std::function<void()>> tasks_;
	std::mutex queue_mutex_;
	std::condition_variable condition_;
	bool stop_requested_;
};
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
	void tf_callback(const tf2_msgs::TFMessageConstPtr &transformStamped);
	geometry_msgs::Transform getCurrent_State();
	bool Planer(moveit::planning_interface::MoveGroupInterface::Plan plan);
	bool asyncExecute();
	bool Execute();
	double round(double num, int exponent);
	geometry_msgs::Pose setPoint(const double x, const double y, const double z);
	geometry_msgs::Pose setPoint(double position[]);
	~MoveitServer();

public:
	std::string reference_frame;
	std::string end_effector_link;
	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroupInterface arm_;
	moveit::planning_interface::MoveGroupInterface::Plan myplan;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener *tfListener;
	// msg
	ros::Publisher tool_do_pub;
	ros::Subscriber tf_sub;
	geometry_msgs::Transform current_state;
	TaskExecutor task_executor_;
};

#endif /* MOVEITSERVER_H_ */

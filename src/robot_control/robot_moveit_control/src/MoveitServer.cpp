#include <MoveitServer.h>

MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP)
{
	// 设置机械臂误差和速度
	arm_.setGoalPositionTolerance(0.001);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.001);
	arm_.setMaxAccelerationScalingFactor(0.1);
	arm_.setMaxVelocityScalingFactor(0.2);
	// 设置规划参数
	const moveit::core::JointModelGroup *joint_model_group = this->arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	this->end_effector_link = arm_.getEndEffectorLink();
	this->reference_frame = "base_link";
	arm_.setPoseReferenceFrame(this->reference_frame);
	arm_.allowReplanning(true);
	arm_.setPlanningTime(5.0);
	arm_.setPlannerId("RRTConnect");
	this->joint_state_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_state", 10, &MoveitServer::joint_state_callback, this);
	// 发布话题消息
	this->tool_do_pub = nh_.advertise<rm_msgs::Tool_Digital_Output>("/rm_driver/Tool_Digital_Output", 10);
	// 夹爪初始化
	this->Set_Tool_DO(1, false);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(2, false);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(1, true);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(1, false);
	ros::Duration(1.0).sleep();
	ROS_INFO("夹爪初始化完成");
}

void MoveitServer::Planer(moveit::planning_interface::MoveGroupInterface::Plan plan) // 规划求解
{
	this->myplan = plan;
	Executer();
}

bool MoveitServer::Executer() // 求解
{
	if (!this->myplan.trajectory_.joint_trajectory.points.empty())
	{
		bool success = (this->arm_.execute(this->myplan) == moveit::core::MoveItErrorCode::SUCCESS); //同步求解
		return success;
	}
	return false;
}

void MoveitServer::go_pose(const std::string str) // 移动到预设位姿
{
	arm_.setNamedTarget(str);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
	}
}

void MoveitServer::joint_state_callback(const sensor_msgs::JointStateConstPtr &msg) // 获取关节状态
{
	this->joint_state->effort = msg.get()->effort;
	this->joint_state->header = msg.get()->header;
	this->joint_state->name = msg.get()->name;
	this->joint_state->position = msg.get()->position;
	this->joint_state->velocity = msg.get()->velocity;
}

bool MoveitServer::move_j(const std::vector<double> &joint_group_positions) // 按目标关节位置移动
{
	arm_.setJointValueTarget(joint_group_positions);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
		return true;
	}
	return false;
}

bool MoveitServer::move_p(const std::vector<double> &pose) // 按目标空间位姿移动(x,y,z,roll,pitch,yaw)
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
		return true;
	}
	return false;
}

bool MoveitServer::move_p(const geometry_msgs::PoseStampedConstPtr &msg) // 按目标空间位姿移动(接收目标物体位姿)
{
	geometry_msgs::Pose target_pose;
	target_pose = msg.get()->pose;
	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
		return true;
	}
	return false;
}

bool MoveitServer::move_p(const std::array<double, 3>& position) // 按目标空间位姿移动(接收x,y,z，保持末端位姿)
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose; 	// keep current pose
	target_pose.orientation = current_pose.orientation;

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
		return true;
	}
	return false;
}

bool MoveitServer::move_l(const std::vector<double> &pose) // 按目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints; // 创建包含到达目标位姿所需的一系列中间点的容器
	geometry_msgs::Pose target_pose;
	target_pose.position.x = pose[0];
	target_pose.position.y = pose[1];
	target_pose.position.z = pose[2];

	tf2::Quaternion myQuaternion;
	myQuaternion.setRPY(pose[3], pose[4], pose[5]);
	target_pose.orientation.x = myQuaternion.getX();
	target_pose.orientation.y = myQuaternion.getY();
	target_pose.orientation.z = myQuaternion.getZ();
	target_pose.orientation.w = myQuaternion.getW();
	
	waypoints.push_back(target_pose); 

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0; // 允许关节空间跳跃最大值
	const double eef_step = 0.01; // 两个连续路径点间的最大移动距离
	double fraction = 0.0; // 规划路径占原始请求路径的比例
	int maxtries = 100; // 最大尝试次数
	int attempts = 0; // 尝试次数

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory); // 计算经过中间点的笛卡尔空间路径
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory; // 计算轨迹传入
		arm_.asyncExecute(plan); // 异步执行规划
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const std::array<double, 3>& position) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;
	target_pose.orientation = current_pose.orientation;

	waypoints.push_back(target_pose);

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100; 
	int attempts = 0;

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		arm_.asyncExecute(plan);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const std::vector<std::vector<double>> &posees) // 按多个目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints;
	for (int i = 0; i < posees.size(); i++)
	{
		geometry_msgs::Pose target_pose;
		target_pose.position.x = posees[i][0];
		target_pose.position.y = posees[i][1];
		target_pose.position.z = posees[i][2];

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(posees[i][3], posees[i][4], posees[i][5]);
		target_pose.orientation.x = myQuaternion.getX();
		target_pose.orientation.y = myQuaternion.getY();
		target_pose.orientation.z = myQuaternion.getZ();
		target_pose.orientation.w = myQuaternion.getW();
		waypoints.push_back(target_pose);
	}

	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;
	int attempts = 0;

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		arm_.asyncExecute(plan);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

void MoveitServer::Set_Tool_DO(int num, bool state) // 控制夹爪开合
{
	rm_msgs::Tool_Digital_Output tool_do_msg; // 创建工具端IO消息
	tool_do_msg.num = num;
	tool_do_msg.state = state;
	tool_do_pub.publish(tool_do_msg);
	ROS_INFO("Published Tool Digital Output message with num = %d and state = %s", num, state ? "true" : "false");
}

MoveitServer::~MoveitServer()
{
	delete this;
}

#include <MoveitServer.h>

MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP)
{

	// 设置机械臂误差和速度
	arm_.setGoalPositionTolerance(0.001);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.001);
	arm_.setMaxAccelerationScalingFactor(0.3);
	arm_.setMaxVelocityScalingFactor(0.5);
	// 设置规划参数
	// const moveit::core::JointModelGroup *joint_model_group = this->arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	this->end_effector_link = arm_.getEndEffectorLink();
	this->tfListener = new tf2_ros::TransformListener(this->tfBuffer);
	this->reference_frame = "base_link";
	arm_.setPoseReferenceFrame(this->reference_frame);
	arm_.allowReplanning(true);
	arm_.setPlanningTime(5.0);
	arm_.setPlannerId("TRRT");
	// this->tf_sub = nh_.subscribe("/tf", 10, &MoveitServer::tf_callback, this);
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
bool MoveitServer::Planer() // 规划求解
{
	bool success = false;
	try
	{
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

		ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
		if (success)
		{
			success = (this->arm_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		}
	}
	catch (const std::exception &e)
	{
		ROS_ERROR("Exception caught while waiting for the asyncPlaner to complete: %s", e.what());
	}

	return success;
}
bool MoveitServer::asyncPlaner() // 规划求解
{
	try
	{
		// 等待上一个异步任务完成
		if (last_task_future.valid())
			last_task_future.get(); // 等待上一个任务完成

		// 异步执行规划和执行动作
		auto future = std::async(std::launch::async, [this]()
								 {
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        if (success)
        {
			success = (this->arm_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        }
        return success; });

		// 更新last_task_future以便跟踪当前任务
		last_task_future = std::move(future);
	}
	catch (const std::exception &e)
	{
		ROS_ERROR("Exception caught while waiting for the asyncPlaner to complete: %s", e.what());
	}

	// 返回异步任务的future，以便在需要时检查任务是否完成或获取结果
	return true;
}

void MoveitServer::go_pose(const std::string str) // 移动到预设位姿
{
	arm_.setNamedTarget(str);
	Planer();
}
geometry_msgs::Pose MoveitServer::setPoint(const double x, const double y, const double z)
{
	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = x;
	target_pose1.position.y = y;
	target_pose1.position.z = z;
	target_pose1.orientation = this->getCurrent_State().rotation;
	return target_pose1;
}
geometry_msgs::Pose MoveitServer::setPoint(const double position[])
{
	geometry_msgs::Pose target_pose1;
	if (sizeof(position) / sizeof(double) == 3)
	{
		target_pose1.position.x = position[0];
		target_pose1.position.y = position[1];
		target_pose1.position.z = position[2];
		target_pose1.orientation = this->getCurrent_State().rotation;
	}
	else if (sizeof(position) / sizeof(double) == 7)
	{
		target_pose1.position.x = position[0];
		target_pose1.position.y = position[1];
		target_pose1.position.z = position[2];
		target_pose1.orientation.x = position[3];
		target_pose1.orientation.y = position[4];
		target_pose1.orientation.z = position[5];
		target_pose1.orientation.w = position[6];
	}

	return target_pose1;
}
double MoveitServer::round(double num, int exponent)
{
	double multiplied = std::round(num * std::pow(10, exponent));
	double result = multiplied / std::pow(10, exponent);
	return result;
}
void MoveitServer::tf_callback(const tf2_msgs::TFMessageConstPtr &tf)
{
	geometry_msgs::TransformStamped transformStamped;

	try
	{
		transformStamped = this->tfBuffer.lookupTransform("base_link", "ee_link",
														  ros::Time(0));

		this->current_state = transformStamped.transform;
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		// ros::Duration(1.0).sleep();
	}
}
geometry_msgs::Transform MoveitServer::getCurrent_State()
{
	geometry_msgs::TransformStamped transformStamped;

	try
	{
		// 尝试获取末端(link6)坐标系到基座坐标系的变换
		transformStamped = this->tfBuffer.lookupTransform("base_link", "ee_link",
														  ros::Time(0));
		// // 输出末端(link6)的姿态信息
		// ROS_INFO("End Effector Pose (base_link->link6):");

		this->current_state = transformStamped.transform;
		return transformStamped.transform;
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		// ros::Duration(1.0).sleep();
	}
	return transformStamped.transform;
}

bool MoveitServer::move_j(const std::vector<double> &joint_group_positions, bool isAsync) // 按目标关节位置移动
{
	arm_.setJointValueTarget(joint_group_positions);
	if (isAsync)
	{
		return asyncPlaner();
	}
	return Planer();
}

bool MoveitServer::move_p(const std::vector<double> &pose, bool isAsync) // 按目标空间位姿移动(x,y,z,roll,pitch,yaw)
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

	arm_.setPoseTarget(target_pose);
	if (isAsync)
	{
		return asyncPlaner();
	}
	return Planer();
}

bool MoveitServer::move_p(const geometry_msgs::Pose &msg, bool isAsync) // 按目标空间位姿移动(接收目标物体位姿)
{
	geometry_msgs::Pose target_pose;
	target_pose = msg;
	arm_.setPoseTarget(target_pose);
	if (isAsync)
	{
		return asyncPlaner();
	}
	return Planer();
}

bool MoveitServer::move_p(const double position[], bool isAsync) // 按目标空间位姿移动(接收x,y,z，保持末端位姿)
{
	geometry_msgs::Pose target_pose;
	target_pose=this->setPoint(position);
	arm_.setPoseTarget(target_pose);
	if (isAsync)
	{
		return asyncPlaner();
	}
	return Planer();
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
	const double eef_step = 0.01;	   // 两个连续路径点间的最大移动距离
	double fraction = 0.0;			   // 规划路径占原始请求路径的比例
	int maxtries = 100;				   // 最大尝试次数
	int attempts = 0;				   // 尝试次数

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
		arm_.asyncExecute(plan);	   // 异步执行规划
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const double (&position)[3]) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	std::vector<geometry_msgs::Pose> waypoints;
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	target_pose.orientation = this->getCurrent_State().rotation;
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

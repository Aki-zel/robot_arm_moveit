#include <MoveitServer.h>

MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP)
{

	// Set arm properties
	arm_.setGoalPositionTolerance(0.001);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.001);
	arm_.setMaxAccelerationScalingFactor(0.3);
	arm_.setMaxVelocityScalingFactor(0.5);
	arm_.setPoseReferenceFrame("base_link");
	arm_.allowReplanning(true);
	arm_.setPlanningTime(3.0);
	arm_.setPlannerId("TRRT");

	tfListener = new tf2_ros::TransformListener(tfBuffer);
	tool_do_pub = nh_.advertise<rm_msgs::Tool_Digital_Output>("/rm_driver/Tool_Digital_Output", 10);
	initializeClaw();
}

bool MoveitServer::Planer() // 规划求解
{
	bool success = false;
	try
	{

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

		if (success)
		{
			ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
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
	bool success = false;
	try
	{

		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
		if (success)
		{
			ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
			// success = (this->arm_.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
			this->arm_.asyncExecute(my_plan);
		}
	}
	catch (const std::exception &e)
	{
		ROS_ERROR("Exception caught while waiting for the asyncPlaner to complete: %s", e.what());
	}

	return success;
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
geometry_msgs::Pose MoveitServer::setPoint(const std::vector<double> &pose)
{
	geometry_msgs::Pose target_pose1;
	if (pose.size() == 3)
	{
		target_pose1.position.x = pose[0];
		target_pose1.position.y = pose[1];
		target_pose1.position.z = pose[2];
		target_pose1.orientation = this->getCurrent_State().rotation;
	}
	else if (pose.size() == 6)
	{
		target_pose1.position.x = pose[0];
		target_pose1.position.y = pose[1];
		target_pose1.position.z = pose[2];
		tf2::Quaternion qua;
		qua.setRPY(pose[3],pose[4],pose[5]);
		target_pose1.orientation.x=qua.getX();
		target_pose1.orientation.y=qua.getY();
		target_pose1.orientation.z=qua.getZ();
		target_pose1.orientation.w=qua.getW();
	}
	else if (pose.size() == 7)
	{
		target_pose1.position.x = pose[0];
		target_pose1.position.y = pose[1];
		target_pose1.position.z = pose[2];
		target_pose1.orientation.x = pose[3];
		target_pose1.orientation.y = pose[4];
		target_pose1.orientation.z = pose[5];
		target_pose1.orientation.w = pose[6];
	}

	return target_pose1;
}
double MoveitServer::round(double num, int exponent)
{
	double multiplied = std::round(num * std::pow(10, exponent));
	double result = multiplied / std::pow(10, exponent);
	return result;
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

void MoveitServer::move_j(const std::vector<double> &joint_group_positions, bool isAsync) // 按目标关节位置移动
{

	arm_.setJointValueTarget(joint_group_positions);
	if (isAsync)
	{
		asyncPlaner();
	}
	Planer();
}
void MoveitServer::stop()
{
	arm_.stop();
}
void MoveitServer::move_p(const std::vector<double> &pose, bool isAsync) // 按目标空间位姿移动(x,y,z,roll,pitch,yaw)
{
	geometry_msgs::Pose target_pose;
	target_pose = this->setPoint(pose);

	arm_.setPoseTarget(target_pose);
	if (isAsync)
	{
		asyncPlaner();
	}
	Planer();
}
void MoveitServer::Set_Tool_DO(int num, bool state) // 控制夹爪开合
{
    rm_msgs::Tool_Digital_Output tool_do_msg; // 创建工具端IO消息
    tool_do_msg.num = num;
    tool_do_msg.state = state;
    tool_do_pub.publish(tool_do_msg);
    ROS_INFO("Published Tool Digital Output message with num = %d and state = %s", num, state ? "true" : "false");
}
void MoveitServer::initializeClaw()
{
    Set_Tool_DO(1, false);
    ros::Duration(1.0).sleep();
    Set_Tool_DO(2, false);
    ros::Duration(1.0).sleep();
    Set_Tool_DO(1, true);
    ros::Duration(1.0).sleep();
    Set_Tool_DO(1, false);
    ros::Duration(1.0).sleep();
    ROS_INFO("Claw initialization completed");
}
void MoveitServer::move_p(const geometry_msgs::Pose &msg, bool isAsync) // 按目标空间位姿移动(接收目标物体位姿)
{
	geometry_msgs::Pose target_pose;
	target_pose = msg;
	arm_.setPoseTarget(target_pose);
	if (isAsync)
	{
		asyncPlaner();
	}
	Planer();
}

void MoveitServer::move_l(const std::vector<double> &pose) // 按目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{

	std::vector<geometry_msgs::Pose> waypoints; // 创建包含到达目标位姿所需的一系列中间点的容器
	geometry_msgs::Pose target_pose;
	target_pose = this->setPoint(pose);
	waypoints.push_back(target_pose);

	this->move_l(waypoints);
}

void MoveitServer::move_l(const std::array<double, 3> &position) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	bool finished = arm_.getMoveGroupClient().waitForResult(ros::Duration(10.0));
	if (finished)
	{
		std::vector<geometry_msgs::Pose> waypoints;
		geometry_msgs::Pose target_pose;
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];

		target_pose.orientation = this->getCurrent_State().rotation;
		waypoints.push_back(target_pose);

		this->move_l(waypoints);
	}
}

void MoveitServer::move_l(const std::vector<std::vector<double>> &posees) // 按多个目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints;
	for (int i = 0; i < posees.size(); i++)
	{
		geometry_msgs::Pose target_pose;
		target_pose = setPoint(posees[i]);
		waypoints.push_back(target_pose);
	}

	this->move_l(waypoints);
}
void MoveitServer::move_l(const std::vector<geometry_msgs::Pose> Points)
{
	moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
	double fraction = 0.0;
	int maxtries = 100;
	int attempts = 0;

	while (fraction < 1.0 && attempts < maxtries)
	{
		fraction = arm_.computeCartesianPath(Points, eef_step, jump_threshold, trajectory);
		attempts++;
	}

	if (fraction == 1)
	{
		ROS_INFO("Path computed successfully. Moving the arm.");
		moveit::planning_interface::MoveGroupInterface::Plan plan;
		plan.trajectory_ = trajectory;
		arm_.execute(plan);
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
	}
}
MoveitServer::~MoveitServer()
{
	delete this;
}

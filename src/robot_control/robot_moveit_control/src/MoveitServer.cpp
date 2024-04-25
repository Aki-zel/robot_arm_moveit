#include <MoveitServer.h>

MoveitServer::MoveitServer(std::string &PLANNING_GROUP):arm_(PLANNING_GROUP)
{
	// 设置机械臂误差和速度
	arm_.setGoalPositionTolerance(0.001);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.001);
	arm_.setMaxAccelerationScalingFactor(0.1);
	arm_.setMaxVelocityScalingFactor(0.2);
	// 设置规划参数
	const moveit::core::JointModelGroup *joint_model_group =
		this->arm_.getCurrentState(3)->getJointModelGroup(PLANNING_GROUP);
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
	ros::Duration(2.0).sleep();
	this->Set_Tool_DO(1, true);
	ros::Duration(2.0).sleep();
	this->Set_Tool_DO(1, false);
	// ros::Duration(2.0).sleep();
	// this->Set_Tool_DO(2, true);
	// ros::Duration(2.0).sleep();
	// this->Set_Tool_DO(2, false);
	ROS_INFO("夹爪初始化完成");
	// MoveitServer::go_home();
}

void MoveitServer::go_home()
{
	arm_.setNamedTarget("home");
	arm_.move();
	sleep(0.5);
}
void MoveitServer::go_pose(const std::string str)
{
	arm_.setNamedTarget(str);
	arm_.move();
	sleep(0.5);
}
void MoveitServer::joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
	this->joint_state->effort = msg.get()->effort;
	this->joint_state->header = msg.get()->header;
	this->joint_state->name = msg.get()->name;
	this->joint_state->position = msg.get()->position;
	this->joint_state->velocity = msg.get()->velocity;
}
bool MoveitServer::move_j(const std::vector<double> &joint_group_positions)
{
	arm_.setJointValueTarget(joint_group_positions);
	arm_.asyncMove();
	sleep(0.5);
	return true;
}

bool MoveitServer::move_p(const std::vector<double> &pose)
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

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_p(const double (&position)[3])
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	// keep current pose
	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;
	target_pose.orientation = current_pose.orientation;

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p:%s", success ? "SUCCESS" : "FAILED");

	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_p_with_constrains(const std::vector<double> &pose)
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

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "Link6";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = myQuaternion.getX();
	ocm.orientation.y = myQuaternion.getY();
	ocm.orientation.z = myQuaternion.getZ();
	ocm.orientation.w = myQuaternion.getW();
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	arm_.setPathConstraints(test_constraints);

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

	arm_.clearPathConstraints();
	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_p_with_constrains(const double (&position)[3])
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "Link6";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = current_pose.orientation.x;
	ocm.orientation.y = current_pose.orientation.y;
	ocm.orientation.z = current_pose.orientation.z;
	ocm.orientation.w = current_pose.orientation.w;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	arm_.setPathConstraints(test_constraints);

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

	arm_.clearPathConstraints();
	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_l(const std::vector<double> &pose)
{

	std::vector<geometry_msgs::Pose> waypoints;
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const double (&position)[3])
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const std::vector<std::vector<double>> &posees)
{
	// plan lots of points
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

void MoveitServer::Set_Tool_DO(int num, bool state)
{
	// 创建工具端IO消息
	rm_msgs::Tool_Digital_Output tool_do_msg;
	tool_do_msg.num = num;
	tool_do_msg.state = state;
	// ros::Duration(1.0).sleep();
	//  发布消息
	tool_do_pub.publish(tool_do_msg);
	ROS_INFO("Published Tool Digital Output message with num = %d and state = %s", num, state ? "true" : "false");
}

MoveitServer::~MoveitServer()
{
	delete this;
}

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
	const moveit::core::JointModelGroup *joint_model_group =
		this->arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
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
	this->Set_Tool_DO(1, true);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(1, false);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(2, false);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(2, true);
	ros::Duration(1.0).sleep();
	this->Set_Tool_DO(2, false);
	ROS_INFO("夹爪初始化完成");
	// MoveitServer::go_home();
}

void MoveitServer::go_home()
{
	arm_.setNamedTarget("home");
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
	}
}
void MoveitServer::Planer(moveit::planning_interface::MoveGroupInterface::Plan plan)
{
	this->myplan = plan;
	Executer();
}
bool MoveitServer::Executer()
{
	if (!this->myplan.trajectory_.joint_trajectory.points.empty())
	{
		bool success = (this->arm_.asyncExecute(this->myplan) == moveit::core::MoveItErrorCode::SUCCESS);
		return success;
	}
	return false;
}
void MoveitServer::go_pose(const std::string str)
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
void MoveitServer::joint_state_callback(const sensor_msgs::JointStateConstPtr &msg)
{
	this->joint_state->effort = msg.get()->effort;
	this->joint_state->header = msg.get()->header;
	this->joint_state->name = msg.get()->name;
	this->joint_state->position = msg.get()->position;
	this->joint_state->velocity = msg.get()->velocity;
}
bool MoveitServer::move_j(const std::vector<double> &joint_group_positions)
{
	arm_.setJointValueTarget(joint_group_positions);
	// arm_.move();
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (arm_.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
	if (success)
	{
		Planer(my_plan);
		return true;
	}
	return false;
	// sleep(0.5);
}

bool MoveitServer::move_p(const std::vector<double> &pose)
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
bool MoveitServer::move_p(const geometry_msgs::PoseStampedConstPtr &msg)
{
	geometry_msgs::Pose target_pose;
	target_pose = msg.get()->pose;
	target_pose.position.x += 0.10;
	target_pose.position.z += 0.18;
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
bool MoveitServer::move_p(const double (&position)[3])
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	// keep current pose
	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;
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

bool MoveitServer::move_p_with_constrains(const std::vector<double> &pose)
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

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "Link6";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = myQuaternion.getX();
	ocm.orientation.y = myQuaternion.getY();
	ocm.orientation.z = myQuaternion.getZ();
	ocm.orientation.w = myQuaternion.getW();
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	arm_.setPathConstraints(test_constraints);

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

	arm_.clearPathConstraints();
	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_p_with_constrains(const double (&position)[3])
{
	geometry_msgs::Pose target_pose;
	target_pose.position.x = position[0];
	target_pose.position.y = position[1];
	target_pose.position.z = position[2];

	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = "Link6";
	ocm.header.frame_id = "base_link";
	ocm.orientation.x = current_pose.orientation.x;
	ocm.orientation.y = current_pose.orientation.y;
	ocm.orientation.z = current_pose.orientation.z;
	ocm.orientation.w = current_pose.orientation.w;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	// Now, set it as the path constraint for the group.
	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	arm_.setPathConstraints(test_constraints);

	arm_.setStartStateToCurrentState();
	arm_.setPoseTarget(target_pose);

	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit::core::MoveItErrorCode success = arm_.plan(plan);
	ROS_INFO("move_p_with_constrains :%s", success ? "SUCCESS" : "FAILED");

	arm_.clearPathConstraints();
	if (success)
	{
		arm_.execute(plan);
		sleep(1);
		return true;
	}
	return false;
}

bool MoveitServer::move_l(const std::vector<double> &pose)
{

	std::vector<geometry_msgs::Pose> waypoints;
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const double (&position)[3])
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

bool MoveitServer::move_l(const std::vector<std::vector<double>> &posees)
{
	// plan lots of points
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

		arm_.execute(plan);
		sleep(1);
		return true;
	}
	else
	{
		ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		return false;
	}
}

void MoveitServer::Set_Tool_DO(int num, bool state)
{
	// 创建工具端IO消息
	rm_msgs::Tool_Digital_Output tool_do_msg;
	tool_do_msg.num = num;
	tool_do_msg.state = state;
	// ros::Duration(1.0).sleep();
	//  发布消息
	tool_do_pub.publish(tool_do_msg);
	ROS_INFO("Published Tool Digital Output message with num = %d and state = %s", num, state ? "true" : "false");
}

MoveitServer::~MoveitServer()
{
	delete this;
}

#include <MoveitServer.h>
MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP), spinner(2)
{
	spinner.start();

	// Set arm properties
	arm_.setGoalPositionTolerance(0.01);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.01);
	arm_.setMaxAccelerationScalingFactor(0.1);
	arm_.setMaxVelocityScalingFactor(0.1);
	arm_.setPoseReferenceFrame("base_link_rm");
	arm_.allowReplanning(true);
	arm_.setPlanningTime(5.0);
	arm_.setPlannerId("TRRT");
	const moveit::core::JointModelGroup *joint_model_group =
		arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	tool_do_pub = nh_.advertise<rm_msgs::Tool_Digital_Output>("/rm_driver/Tool_Digital_Output", 10);
	collision_stage_pub = nh_.advertise<std_msgs::Int16>("/rm_driver/Set_Collision_Stage", 1);
	ROS_INFO_NAMED("tutorial", "Start MOVEITSERVER");
	initializeClaw();
}
void MoveitServer::setMaxVelocity(double speed){
	arm_.setMaxAccelerationScalingFactor(speed);
	arm_.setMaxVelocityScalingFactor(speed);
}
double MoveitServer::degreesToRadians(double degrees)
{
	return round((degrees * M_PI / 180.0), 4);
}
geometry_msgs::Pose MoveitServer::calculateTargetPose(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &trans_pose)
{
	tf2::Transform target_tf;
	tf2::fromMsg(target_pose, target_tf);

	tf2::Transform relative_tf;
	tf2::fromMsg(trans_pose, relative_tf);

	tf2::Transform end_target_tf = target_tf * relative_tf.inverse();

	geometry_msgs::Pose end_target_pose;
	tf2::toMsg(end_target_tf, end_target_pose);
	return end_target_pose;
}
geometry_msgs::Pose MoveitServer::calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform)
{
	tf2::Transform target_tf;
	tf2::fromMsg(target_pose, target_tf);

	tf2::Transform relative_tf;
	tf2::fromMsg(relative_transform, relative_tf);

	tf2::Transform end_target_tf = target_tf * relative_tf.inverse();

	geometry_msgs::Pose end_target_pose;
	tf2::toMsg(end_target_tf, end_target_pose);
	return end_target_pose;
}
geometry_msgs::Pose MoveitServer::transformPose(const geometry_msgs::Pose &pose, const tf2::Transform &transform)
{
	tf2::Transform pose_transform;
	tf2::fromMsg(pose, pose_transform);

	tf2::Transform result_transform = transform * pose_transform;
	geometry_msgs::Pose result_pose;
	tf2::toMsg(result_transform, result_pose);
	return result_pose;
}

geometry_msgs::Pose MoveitServer::moveFromPose(const geometry_msgs::Pose &pose, double distance)
{
	tf2::Transform pose_transform;
	tf2::fromMsg(pose, pose_transform);
	// 构造一个后退的变换矩阵
	tf2::Vector3 backward_vector(0.0, 0.0, distance); // 假设后退方向沿Z轴
	tf2::Vector3 transformed_vector = pose_transform.getBasis() * backward_vector;

	tf2::Transform back_transform;
	back_transform.setIdentity();
	back_transform.setOrigin(transformed_vector);

	// 将目标位置应用后退变换矩阵
	return transformPose(pose, back_transform);
}
bool MoveitServer::Planer() // 规划求解
{
	bool success = false;
	try
	{
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		success = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

		if (success)
		{
			ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
			this->arm_.execute(my_plan);
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
	target_pose1.orientation = getCurrent_State().rotation;
	
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
		target_pose1.orientation = getCurrent_State().rotation;
	}
	else if (pose.size() == 6)
	{
		target_pose1.position.x = pose[0];
		target_pose1.position.y = pose[1];
		target_pose1.position.z = pose[2];
		tf2::Quaternion qua;
		qua.setRPY(pose[3], pose[4], pose[5]);
		target_pose1.orientation.x = qua.getX();
		target_pose1.orientation.y = qua.getY();
		target_pose1.orientation.z = qua.getZ();
		target_pose1.orientation.w = qua.getW();
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

double MoveitServer::round(double num, int exponent) // 四舍五入浮点数
{
	double multiplied = std::round(num * std::pow(10, exponent));
	double result = multiplied / std::pow(10, exponent);
	return result;
}
geometry_msgs::Pose MoveitServer::getCurrent_Pose()
{
	return this->arm_.getCurrentPose().pose;
}
geometry_msgs::Transform MoveitServer::getCurrent_State()
{
	geometry_msgs::TransformStamped transformStamped;

	try
	{
		transformStamped = this->tfBuffer.lookupTransform("base_link_rm", "ee_link", ros::Time(0));
		this->current_state = transformStamped.transform;
		return transformStamped.transform;
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}

	// If the lookupTransform fails, return an identity transform
	geometry_msgs::Transform identity;
	identity.translation.x = 0.0;
	identity.translation.y = 0.0;
	identity.translation.z = 0.0;
	identity.rotation.x = 0.0;
	identity.rotation.y = 0.0;
	identity.rotation.z = 0.0;
	identity.rotation.w = 1.0;
	return identity;
}

void MoveitServer::Set_Tool_DO(int num, bool state) // 控制夹爪开合
{
	rm_msgs::Tool_Digital_Output tool_do_msg;
	tool_do_msg.num = num;
	tool_do_msg.state = state;
	tool_do_pub.publish(tool_do_msg);
	ROS_INFO("Published Tool Digital Output message with num = %d and state = %s", num, state ? "true" : "false");
	ros::Duration(1).sleep();
}

void MoveitServer::initializeClaw()
{
	Set_Tool_DO(1, false);
	Set_Tool_DO(2, false);
	Set_Tool_DO(1, true);
	Set_Tool_DO(1, false);
	Set_Tool_DO(2, true);
	std_msgs::Int16 msg;
	msg.data = 4;
	this->collision_stage_pub.publish(msg);
	ROS_INFO("Collision Stage 4 setup");
	ROS_INFO("Claw initialization completed");
}

void MoveitServer::stop()
{
	arm_.stop();
	arm_.clearPoseTarget();
}
bool MoveitServer::move_j(const std::vector<double> &joint_group_positions, bool succeed) // 按目标关节位置移动
{
	if (succeed)
	{
		arm_.setJointValueTarget(joint_group_positions);
		return Planer();
	}
	return succeed;
}

bool MoveitServer::move_p(const std::vector<double> &pose, bool succeed) // 按目标空间位姿移动(x,y,z,roll,pitch,yaw)
{
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = this->setPoint(pose);

		arm_.setPoseTarget(target_pose);
		return Planer();
	}
	return succeed;
}
bool MoveitServer::move_p(const geometry_msgs::Pose &msg, bool succeed) // 按目标空间位姿移动(接收目标物体位姿)
{
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = msg;
		arm_.setPoseTarget(target_pose);
		return Planer();
	}
	return succeed;
}
bool MoveitServer::move_p(const geometry_msgs::PoseStamped &msg, bool succeed) // 按目标空间位姿移动(接收目标物体位姿)
{
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = msg.pose;
		arm_.setPoseTarget(target_pose);
		return Planer();
	}
	return succeed;
}

bool MoveitServer::move_l(const std::vector<double> &pose, bool succeed) // 按目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{

		geometry_msgs::Pose target_pose;
		target_pose = this->setPoint(pose);
		waypoints.push_back(target_pose);

		return this->move_l(waypoints);
	}
	return succeed;
}

bool MoveitServer::move_l(const std::array<double, 3> &position, bool succeed) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose.position.x = position[0];
		target_pose.position.y = position[1];
		target_pose.position.z = position[2];

		target_pose.orientation = getCurrent_State().rotation;
		waypoints.push_back(target_pose);

		return this->move_l(waypoints);
	}
	return succeed;
}
bool MoveitServer::move_l(const geometry_msgs::Pose &position, bool succeed) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{
		waypoints.push_back(position);

		return this->move_l(waypoints);
	}
	return succeed;
}

bool MoveitServer::move_l(const std::vector<std::vector<double>> &posees, bool succeed) // 按多个目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{
		for (int i = 0; i < posees.size(); i++)
		{
			geometry_msgs::Pose target_pose;
			target_pose = setPoint(posees[i]);
			waypoints.push_back(target_pose);
		}

		return this->move_l(waypoints);
	}
	return succeed;
}

bool MoveitServer::move_l(const std::vector<geometry_msgs::Pose> Points, bool succeed)
{
	moveit_msgs::RobotTrajectory trajectory;
	if (succeed)
	{
		const double jump_threshold = 0.000;
		const double eef_step = 0.01;
		double fraction = 0.0;
		int maxtries = 10;
		int attempts = 0;

		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_.computeCartesianPath(Points, eef_step, jump_threshold, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");
			// moveit::planning_interface::MoveGroupInterface::Plan plan;
			// plan.trajectory_ = trajectory;
			arm_.execute(trajectory);
			return true;
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
		}
	}
	return succeed;
}

MoveitServer::~MoveitServer()
{
	delete tfListener;
}

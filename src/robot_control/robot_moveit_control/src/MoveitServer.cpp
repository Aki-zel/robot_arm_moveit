#include <MoveitServer.h>
#include <rm_msgs/MoveL.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP), spinner(3)
{
	spinner.start();
	// if (!arm_.getCurrentState())
	// {
	// 	ROS_ERROR("Error: MoveGroupInterface current state is not initialized.");
	// 	return;
	// }
	// Set arm properties
	arm_.setGoalPositionTolerance(0.01);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.01);
	arm_.setMaxAccelerationScalingFactor(0.1);
	arm_.setMaxVelocityScalingFactor(0.1);
	arm_.setPoseReferenceFrame("base_link_rm");
	arm_.allowReplanning(true);
	arm_.setPlanningTime(5.0);
	// arm_.setPlannerId("LBTRRT");
	// arm_.setPlannerId("TRRT");
	arm_.setPlannerId("PTP");
	// arm_.setPlannerId("RRTConnect");
	// arm_.setPlannerId("RRTstar");
	// arm_.setEndEffectorLink("ee_link");
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	tool_do_pub = nh_.advertise<rm_msgs::Tool_Digital_Output>("/rm_driver/Tool_Digital_Output", 10);
	collision_stage_pub = nh_.advertise<std_msgs::Int16>("/rm_driver/Set_Collision_Stage", 1);
	moveL_cmd = nh_.advertise<rm_msgs::MoveL>("/rm_driver/MoveL_Cmd", 10);
	joint_model_group = arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	ROS_INFO_NAMED("tutorial", "Start MOVEITSERVER");
	// ROS_INFO("%s", arm_.getEndEffectorLink().c_str());
	// visual_tools = new moveit_visual_tools::MoveItVisualTools("base_link");
	// visual_tools->deleteAllMarkers();
	// visual_tools->loadRemoteControl();
	// end_effector_link = arm_.getRobotModel()->getLinkModel(arm_.getEndEffectorLink());
	initializeClaw();
}
void MoveitServer::setMaxVelocity(double speed)
{
	arm_.setMaxAccelerationScalingFactor(speed);
	arm_.setMaxVelocityScalingFactor(speed);
}
bool MoveitServer::Planer() // 规划求解
{
	bool success = false;
	// visual_tools->deleteAllMarkers();
	try
	{
		moveit::planning_interface::MoveGroupInterface::Plan my_plan;

		success = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		if (success)
		{
			ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
			this->arm_.execute(my_plan);
			// robot_trajectory::RobotTrajectory trajectory_(arm_.getRobotModel());
			// trajectory_.setRobotTrajectoryMsg(*arm_.getCurrentState(), my_plan.trajectory_);
			// visual_tools->publishTrajectoryLine(trajectory_, end_effector_link);
			// visual_tools->trigger();
			// visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue");
		}
		arm_.clearPathConstraints();
	}
	catch (const std::exception &e)
	{
		ROS_ERROR("Exception caught while waiting for the asyncPlaner to complete: %s", e.what());
	}

	return success;
}
bool MoveitServer::go_home() // 移动到预设位姿
{
	arm_.setNamedTarget("zero");
	return Planer();
}
bool MoveitServer::go_pose(const std::string str) // 移动到预设位姿
{
	arm_.setNamedTarget(str);
	return Planer();
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


geometry_msgs::Pose MoveitServer::getCurrent_Pose()
{
	geometry_msgs::PoseStamped current_pose;

	try
	{
		current_pose = this->arm_.getCurrentPose(arm_.getEndEffectorLink());
		// 获取 base 到 base_rm 的转换
		geometry_msgs::TransformStamped transformStamped = this->tfBuffer.lookupTransform("base_link_rm", "base_link", ros::Time(0));

		// 将姿态从 base 转换到 base_rm
		tf2::doTransform(current_pose, current_pose, transformStamped);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		return current_pose.pose; // 如果转换失败，返回原始姿态
	}

	return current_pose.pose;
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
void MoveitServer::setCollisionMatrix()
{
	const moveit::core::RobotModelConstPtr &kinematic_model = arm_.getRobotModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
	acm.setEntry("gripper_link", true);
	acm.setEntry("ee_link", true);
	moveit_msgs::PlanningScene scene;
	planning_scene.getPlanningSceneMsg(scene);
	planning_scene_interface.applyPlanningScene(scene);
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
	arm_.setNamedTarget("zero");
	arm_.asyncMove();
	Set_Tool_DO(1, false);
	Set_Tool_DO(2, false);
	Set_Tool_DO(1, true);
	Set_Tool_DO(1, false);
	Set_Tool_DO(2, true);
	Set_Tool_DO(2, false);
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
void MoveitServer::getCurrentJoint(std::vector<double> &joint_group_positions)
{
	moveit::core::RobotStatePtr current_state = arm_.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
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
bool MoveitServer::move_p_with_constrains(geometry_msgs::Pose &target_pose, bool succeed)
{
	// geometry_msgs::Pose target_pose;
	// target_pose.position.x = position[0];
	// target_pose.position.y = position[1];
	// target_pose.position.z = position[2];

	// geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;
	if (succeed)
	{
		moveit_msgs::OrientationConstraint ocm;
		ocm.link_name = arm_.getEndEffectorLink();
		ocm.header.frame_id = arm_.getPoseReferenceFrame();
		ocm.orientation = target_pose.orientation;
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
	// visual_tools->deleteAllMarkers();
	if (succeed)
	{
		const double jump_threshold = 15.0;
		const double eef_step = 0.005;
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
			robot_trajectory::RobotTrajectory rt(arm_.getCurrentState()->getRobotModel(), "arm");
			rt.setRobotTrajectoryMsg(*arm_.getCurrentState(), trajectory);

			trajectory_processing::IterativeParabolicTimeParameterization iptp;
			bool success = iptp.computeTimeStamps(rt, 0.1, 0.1); // 速度和加速度缩放因子
			trajectory_processing::IterativeSplineParameterization ipp;
			success = ipp.computeTimeStamps(rt, 1, 1); // 速度和加速度缩放因子
			//   trajectory_processing::TimeOptimalTrajectoryGeneration totg;
			// bool success = totg.computeTimeStamps(rt); // 使用 TOTG 方法

			if (success)
			{
				// ROS_INFO("Time parametrization succeeded");
				rt.getRobotTrajectoryMsg(trajectory);

				moveit::planning_interface::MoveGroupInterface::Plan plan;
				plan.trajectory_ = trajectory;
				arm_.execute(plan);
				// visual_tools->prompt("Press 'next' in the RvizVisualToolsGui window to continue");
				return true;
			}
		}
		else
		{
			ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
			return false;
		}
	}
	return succeed;
}

MoveitServer::~MoveitServer()
{
	delete tfListener;
}

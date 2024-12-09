#include <MoveitServer.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <geometry_msgs/Point.h>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <sensor_msgs/JointState.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <industrial_trajectory_filters/uniform_sample_filter.h>
#include <robotTool.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <moveit_msgs/GetMotionSequence.h>
/// @brief 构造函数
/// @param PLANNING_GROUP
MoveitServer::MoveitServer(std::string &PLANNING_GROUP) : arm_(PLANNING_GROUP), spinner(3)
{
	setlocale(LC_ALL, ""); // 设置编码
	spinner.start();
	tool = std::make_unique<robotTool>();
	plan_group = PLANNING_GROUP;
	arm_.setGoalPositionTolerance(0.001);
	arm_.setGoalOrientationTolerance(0.01);
	arm_.setGoalJointTolerance(0.01);
	arm_.setMaxAccelerationScalingFactor(0.1);
	arm_.setMaxVelocityScalingFactor(0.1);
	// arm_.setPoseReferenceFrame("base_link_rm");
	arm_.allowReplanning(true);
	arm_.setPlanningTime(5.0);
	// arm_.setPlannerId("LBTRRT");
	// arm_.setPlanningPipelineId("ompl");
	// arm_.setPlannerId("TRRT");
	// arm_.setPlanningPipelineId("pilz_industrial_motion_planner");
	// arm_.setPlannerId("PTP");
	arm_.setPlannerId("RRTConnect");
	// arm_.setPlannerId("RRTstar");
	// arm_.setEndEffectorLink("ee_link");
	planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
	tfListener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
	clear_octomap = nh_.serviceClient<std_srvs::Empty>("/clear_octomap");
	joint_model_group = arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	joint_sub_ = nh_.subscribe("joint_positions", 10, &MoveitServer::jointCallback, this);
	pose_name_sub_ = nh_.subscribe("pose_name", 10, &MoveitServer::poseNameCallback, this);
	pose_service_ = nh_.advertiseService("get_end_effector_pose", &MoveitServer::getEndEffectorPoseService, this);
	arm_setting_sub_ = nh_.subscribe("arm_setting", 10, &MoveitServer::armSettingCallback, this);
	arm_pose_sub_ = nh_.subscribe("arm_pose", 10, &MoveitServer::armPoseCallback, this);
	pose_subscriber_ = nh_.subscribe("/arm_pose_goal", 10, &MoveitServer::poseCallback, this);
	timer_ = nh_.createTimer(ros::Duration(3.0), &MoveitServer::clearOctoMapCallback, this);
	ROS_INFO_NAMED("tutorial", "Start MOVEITSERVER");
	// initializeClaw();
}
/// @brief 设置最大速度和加速度
/// @param speed
void MoveitServer::setMaxVelocity(double vel, double acc)
{
	arm_.setMaxAccelerationScalingFactor(acc);
	arm_.setMaxVelocityScalingFactor(vel);
}
void MoveitServer::setclearOctomap(const bool enable)
{
	enable_octomap_=enable;
}
void MoveitServer::clearOctoMapCallback(const ros::TimerEvent&){
	if (enable_octomap_) {
            std_srvs::Empty srv;
            if (clear_octomap.call(srv)) {
                // ROS_INFO("OctoMap cleared successfully.");
            } else {
                ROS_ERROR("Failed to call service /clear_octomap.");
            }
        }
}
/// @brief 规划求解并执行运动
/// @return True表示成功规划并执行，False表示规划失败 ?
bool MoveitServer::Planer()
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
		arm_.clearPathConstraints();
	}
	catch (const std::exception &e)
	{
		ROS_ERROR("Exception caught while waiting for the asyncPlaner to complete: %s", e.what());
	}

	return success;
}
/// @brief 回到起始点
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::go_home() // 移动到预设位姿
{
	arm_.setNamedTarget("zero");
	return Planer();
}
/// @brief 回到预设点
/// @param str
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::go_pose(const std::string str) // 移动到预设位姿
{
	arm_.setNamedTarget(str);
	return Planer();
}

/// @brief 将XYZ转化为pose,并使用机械臂当前姿态
/// @param x
/// @param y
/// @param z
/// @return pose类型的数据点
geometry_msgs::Pose MoveitServer::setPoint(const double x, const double y, const double z)
{
	geometry_msgs::Pose target_pose1;
	target_pose1.position.x = x;
	target_pose1.position.y = y;
	target_pose1.position.z = z;
	target_pose1.orientation = getCurrent_State().rotation;

	return target_pose1;
}

/// @brief 将数组数据转化为pose点，根据数组数据的长度进行判断
/// @param pose 长度3表示XYZ;长度6表示XYZ,RPY;长度7表示XYZ,QXQYQZQW
/// @return 对应数组长度的转化的pose点
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

/// @brief 获取机械臂当前末端pose数据
/// @return 当前机械臂末端pose
geometry_msgs::Pose MoveitServer::getCurrent_Pose()
{
	geometry_msgs::PoseStamped current_pose;

	try
	{
		current_pose = this->arm_.getCurrentPose(arm_.getEndEffectorLink());
		// 获取 base 到 base_rm 的转换
		// geometry_msgs::TransformStamped transformStamped = this->tfBuffer.lookupTransform("base_link_rm", "base_link", ros::Time(0));

		// // 将姿态从 base 转换到 base_rm
		// tf2::doTransform(current_pose, current_pose, transformStamped);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
		return current_pose.pose; // 如果转换失败，返回原始姿态
	}

	return current_pose.pose;
}
/// @brief 获取机械臂当前末端的变化关系
/// @return 当前机械臂末端变化关系
geometry_msgs::Transform MoveitServer::getCurrent_State()
{
	geometry_msgs::TransformStamped transformStamped;

	try
	{
		transformStamped = this->tf_buffer_->lookupTransform("xMate3_base", "tool", ros::Time(0));
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
/// @brief 设置碰撞矩阵，令末端无视碰撞
void MoveitServer::setCollisionMatrix()
{
	const moveit::core::RobotModelConstPtr &kinematic_model = arm_.getRobotModel();
	planning_scene::PlanningScene planning_scene(kinematic_model);
	collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
	acm.setEntry("gripper_link", true);
	acm.setEntry("ee_link", true);
	moveit_msgs::PlanningScene scene;
	planning_scene.getPlanningSceneMsg(scene);
	planning_scene_interface->applyPlanningScene(scene);
}
/// @brief 初始化夹爪并将机械臂回到零位
void MoveitServer::initializeClaw()
{
	// move_j(std::vector<double>{tool->degreesToRadians(0), tool->degreesToRadians(0), tool->degreesToRadians(-90),
	// 						   tool->degreesToRadians(0), tool->degreesToRadians(-90), tool->degreesToRadians(0)});
	go_home();
}

/// @brief 停止当前运动并清空目标
void MoveitServer::stop()
{
	arm_.stop();
	arm_.clearPoseTarget();
}
/// @brief 获取当前机械臂各关节角度 单位RAD
/// @param joint_group_positions 用于传参的数组
/// @return 返回各关节矩阵
void MoveitServer::getCurrentJoint(std::vector<double> &joint_group_positions)
{
	moveit::core::RobotStatePtr current_state = arm_.getCurrentState();
	current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}
bool MoveitServer::move_j(const double j1, const double j2, const double j3, const double j4, const double j5, const double j6, bool succeed)
{
	if (succeed)
	{
		std::vector<double> joint_group_positions = {tool->degreesToRadians(j1), tool->degreesToRadians(j2), tool->degreesToRadians(j3),
													 tool->degreesToRadians(j4), tool->degreesToRadians(j5), tool->degreesToRadians(j6)};
		arm_.setJointValueTarget(joint_group_positions);
		return Planer();
	}
	return succeed;
}
/// @brief 根据各关节进行关节空间运动
/// @param joint_group_positions 单位RAD
/// @param succeed 是否运行该函数
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_j(const std::vector<double> &joint_group_positions, bool succeed) // 按目标关节位置移动
{
	if (succeed)
	{
		arm_.setJointValueTarget(joint_group_positions);
		return Planer();
	}
	return succeed;
}

/// @brief 笛卡尔空间坐标运动
/// @param pose 用于调用setpoint的数组
/// @param succeed 是否运行该函数
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_p(const std::vector<double> &pose, bool succeed) // 按目标空间位姿移动(x,y,z,roll,pitch,yaw)
{
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = this->setPoint(pose);

		// arm_.setPoseTarget(target_pose);
		return move_p(target_pose);
	}
	return succeed;
}
/// @brief 笛卡尔空间坐标运动
/// @param msg posestamped
/// @param succeed 是否运行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_p(const geometry_msgs::PoseStamped &msg, bool succeed) // 按目标空间位姿移动(接收目标物体位姿)
{
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = msg.pose;
		// arm_.setPoseTarget(target_pose);
		return move_p(target_pose);
	}
	return succeed;
}
/// @brief 笛卡尔空间坐标运动
/// @param msg 笛卡尔空间坐标点
/// @param succeed 是否运行该语句
/// @return True表示成功规划并执行，False表示规划失败
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
/// @brief 基于规划的笛卡尔空间直线运动
/// @param msg pose
/// @param succeed 是否执行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_p_l(const geometry_msgs::Pose &msg, bool succeed)
{
	bool s;
	arm_.setPlannerId("LIN");
	if (succeed)
	{
		geometry_msgs::Pose target_pose;
		target_pose = msg;
		arm_.setPoseTarget(target_pose);
		s = Planer();
	}
	arm_.setPlannerId("PTP");
	return s;
}
void MoveitServer::setConstraint(const moveit_msgs::Constraints cons)
{
	arm_.setPathConstraints(cons);
}

/// @brief 笛卡尔空间直线运动
/// @param pose 用于setpoint的点数据
/// @param succeed 是否运行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_l(const std::vector<double> &pose, bool succeed, bool is_async) // 按目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{

		geometry_msgs::Pose target_pose;
		target_pose = this->setPoint(pose);
		waypoints.push_back(target_pose);

		return this->move_l(waypoints, succeed, is_async);
	}
	return succeed;
}

/// @brief 笛卡尔空间直线运动
/// @param position 目标点pose
/// @param succeed 是否执行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_l(const geometry_msgs::Pose &position, bool succeed, bool is_async) // 按目标空间位姿直线移动(接收x,y,z，保持末端位姿)
{
	std::vector<geometry_msgs::Pose> waypoints;
	if (succeed)
	{
		waypoints.push_back(position);

		return this->move_l(waypoints, succeed, is_async);
	}
	return succeed;
}

/// @brief 笛卡尔空间直线运动
/// @param posees 用于setpoint的多组点数据
/// @param succeed 是否执行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_l(const std::vector<std::vector<double>> &posees, bool succeed, bool is_async) // 按多个目标空间位姿走直线移动(x,y,z,roll,pitch,yaw)
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

		return this->move_l(waypoints, succeed, is_async);
	}
	return succeed;
}

/// @brief 笛卡尔空间直线运动
/// @param Points 多个pose点组成的数组
/// @param succeed 是否运行该语句
/// @return True表示成功规划并执行，False表示规划失败
bool MoveitServer::move_l(const std::vector<geometry_msgs::Pose> Points, bool succeed, bool is_async)
{
	moveit_msgs::RobotTrajectory trajectory;
	if (succeed)
	{
		const double jump_threshold = 10;
		const double eef_step = 0.001;
		double fraction = 0.0;
		int maxtries = 10;
		int attempts = 0;
		while (fraction < 1.0 && attempts < maxtries)
		{
			fraction = arm_.computeCartesianPath(Points, eef_step, trajectory);
			attempts++;
		}

		if (fraction == 1)
		{
			ROS_INFO("Path computed successfully. Moving the arm.");
			moveit::core::RobotModelConstPtr robot_model = arm_.getCurrentState()->getRobotModel();
			robot_trajectory::RobotTrajectory rt(robot_model, plan_group);
			rt.setRobotTrajectoryMsg(*arm_.getCurrentState(), trajectory);
			bool success = true;
			// trajectory_processing::IterativeParabolicTimeParameterization iptp;
			// // // 速度和加速度缩放因子
			// success = iptp.computeTimeStamps(rt, 0.8, 0.6);
			// trajectory_processing::IterativeSplineParameterization ipp;
			// success = ipp.computeTimeStamps(rt, 1, 1); // 速度和加速度缩放因子
			trajectory_processing::TimeOptimalTrajectoryGeneration totg;
			success = totg.computeTimeStamps(rt, 0.8, 0.5); // 速度和加速度缩放因子
			if (success)
			{
				rt.getRobotTrajectoryMsg(trajectory);
				industrial_trajectory_filters::MessageAdapter msg_adapter;
				msg_adapter.request.trajectory = trajectory.joint_trajectory;
				industrial_trajectory_filters::UniformSampleFilter<industrial_trajectory_filters::MessageAdapter> uniform_sample_filter;
				uniform_sample_filter.configure();

				industrial_trajectory_filters::MessageAdapter filtered_msg_adapter;
				if (uniform_sample_filter.update(msg_adapter, filtered_msg_adapter))
				{
					rt.setRobotTrajectoryMsg(*arm_.getCurrentState(), filtered_msg_adapter.request.trajectory);
				}
				else
				{
					ROS_WARN("Uniform sample filter failed.");
				}
				rt.getRobotTrajectoryMsg(trajectory);
				moveit::planning_interface::MoveGroupInterface::Plan plan;
				plan.trajectory_ = trajectory;
				if (is_async)
				{
					arm_.asyncExecute(plan);
				}
				else
				{
					arm_.execute(plan);
				}
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
void MoveitServer::jointCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
	std::vector<double> joint_positions(msg->data.begin(), msg->data.end());
	move_j(joint_positions); // 调用 move_j 接口
}
void MoveitServer::poseNameCallback(const std_msgs::String::ConstPtr &msg)
{
	std::string pose_name = msg->data;
	ROS_INFO("Received pose name: %s", pose_name.c_str());

	// 调用 go_pose 函数执行姿态转换
	if (!go_pose(pose_name))
	{
		ROS_WARN("Failed to execute go_pose for pose: %s", pose_name.c_str());
	}
}
bool MoveitServer::getEndEffectorPoseService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;

	// 将末端姿态信息转换为字符串返回，或者通过其他方式传输具体位姿信息
	std::stringstream ss;
	ss << "Position: " << std::endl
	   << "X:" << current_pose.position.x << std::endl
	   << "Y:" << current_pose.position.y << std::endl
	   << "Z:" << current_pose.position.z << std::endl
	   << "Orientation: " << std::endl
	   << "OX:" << current_pose.orientation.x << std::endl
	   << "OY:" << current_pose.orientation.y << std::endl
	   << "OZ:" << current_pose.orientation.z << std::endl
	   << "OW:" << current_pose.orientation.w;
	res.message = ss.str();
	res.success = true;

	return true;
}
void MoveitServer::armSettingCallback(const robot_msgs::ArmSetting::ConstPtr &msg)
{
	float vel = msg->vel;
	float acc = msg->acc;
	setMaxVelocity(vel, acc);
}
void MoveitServer::armPoseCallback(const robot_msgs::ArmPose::ConstPtr &msg)
{
	// 创建geometry_msgs::Pose并填充数据
	geometry_msgs::Pose target_pose, move_pose;
	target_pose.position.x = msg->x;
	target_pose.position.y = msg->y;
	target_pose.position.z = msg->z;

	// 使用tf2创建姿态四元数
	tf2::Quaternion quaternion;
	quaternion.setRPY(msg->roll, msg->pitch, msg->yaw);
	target_pose.orientation = tf2::toMsg(quaternion);
	geometry_msgs::Pose current_pose = arm_.getCurrentPose().pose;
	move_pose = tool->calculateTargetPose(current_pose, target_pose);
	// arm_.stop();
	// 调用move_l函数，控制机械臂移动到指定位置
	move_l(move_pose);
}
void MoveitServer::poseCallback(const geometry_msgs::Pose::ConstPtr &pose_msg)
{
	// 更新机械臂目标位姿并执行运动
	geometry_msgs::Pose pose;
	pose.orientation = pose_msg.get()->orientation;
	pose.position = pose_msg.get()->position;
	move_p(pose);
}
MoveitServer::~MoveitServer()
{
	// delete tfListener;
}

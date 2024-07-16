
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <string>
#include <iostream>
#include <MoveitServer.h>
#include <robot_msgs/Hand_Catch.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <robotTool.h>
using namespace std;
int main(int argc, char **argv)
{
	robotTool tool;
	// 设置编码
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "moveit_control_server_cpp");
	ros::AsyncSpinner spinner(1);
	ros::NodeHandle nh;
	spinner.start();
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	std::string PLANNING_GROUP = "arm";
	// moveit::planning_interface::MoveGroupInterface arm(PLANNING_GROUP);
	MoveitServer arm(PLANNING_GROUP);
	// ros::ServiceClient client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect"); // 创建目标检测服务客户端
	// arm.move_j(std::vector<double>{arm.degreesToRadians(28), arm.degreesToRadians(-52), arm.degreesToRadians(-99),
	// 										 arm.degreesToRadians(124), arm.degreesToRadians(95), arm.degreesToRadians(155)});
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_link_rm");
	visual_tools.deleteAllMarkers();
	arm.go_pose("zero");
	geometry_msgs::Pose pose, pose2, pose3;
	geometry_msgs::Pose pose1, current_pose;
	std::vector<double> pos = {0.20, 0, 0.30, -3.1415, 0, 0};
	pose = arm.setPoint(pos);
	// pose1 = arm.calculateTargetPose(pose, arm.setPoint(std::vector<double>{0, 0, -0.1, 0, 0, 0}));
	// pose2 = calculateTargetPose(pose, arm.setPoint(std::vector<double>{0, 0, -0.1, 0, 0, 0}));
	// pose1 = arm.calculateTargetPose(pose1, arm.setPoint(std::vector<double>{0, 0, 0, arm.degreesToRadians(90), 0, 0}));
	// pose3 = calculateTargetPose(pose2, arm.setPoint(std::vector<double>{0, 0, 0, arm.degreesToRadians(90), 0, 0}));
	// publishStaticTFwithRot(pose1, "pose1");
	// publishStaticTFwithRot(pose2, "pose2");
	// publishStaticTFwithRot(pose3, "pose3");
	arm.move_p(pose);
	current_pose = arm.getCurrent_Pose();
	// moveit_msgs::PositionConstraint plane_constraint;
	// plane_constraint.header.frame_id = arm.arm_.getPoseReferenceFrame();
	// plane_constraint.link_name = arm.arm_.getEndEffectorLink();
	// shape_msgs::SolidPrimitive plane;
	// plane.type = shape_msgs::SolidPrimitive::BOX;
	// plane.dimensions = {0.0005, 0.0005, 1.0};
	// plane_constraint.constraint_region.primitives.emplace_back(plane);
	// geometry_msgs::Pose constraint_pose;
	// constraint_pose.position = pose.position;
	// constraint_pose.orientation = pose.orientation;
	// plane_constraint.constraint_region.primitive_poses.emplace_back(constraint_pose);
	// plane_constraint.weight = 1.0;
	// moveit_msgs::OrientationConstraint orientation_constraint;
	// orientation_constraint.header.frame_id = arm.arm_.getPoseReferenceFrame();
	// orientation_constraint.link_name = arm.arm_.getEndEffectorLink();

	// orientation_constraint.orientation = pose.orientation;
	// orientation_constraint.absolute_x_axis_tolerance = 0.4;
	// orientation_constraint.absolute_y_axis_tolerance = 0.4;
	// orientation_constraint.absolute_z_axis_tolerance = 0.4;
	// orientation_constraint.weight = 1.0;

	// moveit_msgs::Constraints orientation_constraints;
	// orientation_constraints.orientation_constraints.emplace_back(orientation_constraint);
	// moveit_msgs::Constraints plane_constraints;
	// // plane_constraints.position_constraints.emplace_back(plane_constraint);
	//  plane_constraints.orientation_constraints.emplace_back(orientation_constraint);
	// plane_constraints.name = "use_equality_constraints";
	// arm.arm_.setPathConstraints(plane_constraints);
	// visual_tools.publishLine(current_pose.position, pose1.position, rviz_visual_tools::TRANSLUCENT_DARK);
	// visual_tools.trigger();
	// arm.move_l_cmd(pose1);
	geometry_msgs::TransformStamped transformStamped;
	try
	{
		// arm.setCollisionMatrix();
		// geometry_msgs::TransformStamped transform_base_to_camera;
		// geometry_msgs::TransformStamped transform_base_to_ee;
		transformStamped = tfBuffer.lookupTransform("camera_color_optical_frame", "ee_link",
													ros::Time(0));
		pose1 = tool.calculateTargetTransform(pose, transformStamped.transform);
		arm.move_p(pose1);
		// geometry_msgs::Pose link6_target_pose = calculateTargetTransform(pose, transformStamped.transform);
		// tf2::fromMsg(transformStamped.transform, transform);
		// geometry_msgs::Pose trans_pose;
		// trans_pose = arm.transformPose(pose, transform);
		// trans_pose = arm.moveFromPose(pose, -0.1);
		// arm.move_p(trans_pose);
		// arm.move_p(pose);
		// arm.move_l(waypoints);
		// arm.move_p(link6_target_pose);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}

	ros::waitForShutdown();
	return 0;
}

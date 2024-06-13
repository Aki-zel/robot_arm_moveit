
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>
using namespace std;
// geometry_msgs::Pose calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform)
// {
// 	// Create transformation matrices
// 	tf2::Quaternion q1(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
// 	tf2::Matrix3x3 m1(q1);
// 	tf2::Vector3 t1(target_pose.position.x, target_pose.position.y, target_pose.position.z);
// 	tf2::Quaternion q2(relative_transform.rotation.x, relative_transform.rotation.y, relative_transform.rotation.z, relative_transform.rotation.w);
// 	tf2::Matrix3x3 m2(q2);
// 	tf2::Vector3 t2(relative_transform.translation.x, relative_transform.translation.y, relative_transform.translation.z);

// 	// Calculate link6 target transform
// 	tf2::Matrix3x3 link6_target_matrix = m1 * m2.inverse();
// 	tf2::Vector3 link6_target_position = t1 - (m1 * m2.inverse()) * t2;

// 	tf2::Quaternion link6_target_orientation;
// 	link6_target_matrix.getRotation(link6_target_orientation);

// 	// Convert back to Pose
// 	geometry_msgs::Pose link6_target_pose;
// 	link6_target_pose.position.x = link6_target_position.x();
// 	link6_target_pose.position.y = link6_target_position.y();
// 	link6_target_pose.position.z = link6_target_position.z();
// 	link6_target_pose.orientation.x = link6_target_orientation.x();
// 	link6_target_pose.orientation.y = link6_target_orientation.y();
// 	link6_target_pose.orientation.z = link6_target_orientation.z();
// 	link6_target_pose.orientation.w = link6_target_orientation.w();

// 	return link6_target_pose;
// }

void generateCirclePoints(const geometry_msgs::Pose &center_pose, double radius, double step_angle_degrees, std::vector<geometry_msgs::Pose> &waypoints)
{
	int num_points = static_cast<int>(360.0 / step_angle_degrees);
	for (int i = 0; i < num_points; ++i)
	{
		double angle = step_angle_degrees * i * 3.1415926 / 180.0;
		geometry_msgs::Pose target_pose = center_pose;

		target_pose.position.x = center_pose.position.x + radius * cos(angle);
		target_pose.position.y = center_pose.position.y + radius * sin(angle);

		// // 计算指向圆心的方向
		// tf2::Vector3 to_center(center_pose.position.x - target_pose.position.x, center_pose.position.y - target_pose.position.y, 0);
		// tf2::Vector3 default_dir(1, 0, 0); // 默认方向

		// // 计算四元数使其朝向圆心
		// tf2::Quaternion quat;
		// quat.setRotation(default_dir.cross(to_center).normalized(), std::acos(default_dir.dot(to_center.normalized())));

		// target_pose.orientation = tf2::toMsg(quat);
		waypoints.push_back(target_pose);
	}
}

geometry_msgs::Pose calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform)
{
	// Create transformation from target_pose
	tf2::Transform target_tf;
	tf2::fromMsg(target_pose, target_tf);

	// Create transformation from relative_transform
	tf2::Transform relative_tf;
	tf2::fromMsg(relative_transform, relative_tf);

	// Calculate link6 target transform
	tf2::Transform end_target_tf = target_tf * relative_tf.inverse();

	// Convert back to Pose
	geometry_msgs::Pose end_target_pose;
	tf2::toMsg(end_target_tf, end_target_pose);
	return end_target_pose;
}
int main(int argc, char **argv)
{
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
	geometry_msgs::Pose pose;
	geometry_msgs::Pose pose1;
	tf2::Quaternion qua;
	// qua.setRPY(-3.1415, 0, 0);s
	// pose.position.x = 0.24;
	// pose.position.y = 0;
	// pose.position.z = 0.2;
	// pose.orientation.x = qua.getX();
	// pose.orientation.y = qua.getY();
	// pose.orientation.z = qua.getZ();
	// pose.orientation.w = qua.getW();
	std::vector<double> pos = {0.2, 0, 0.20, -3.1415, 0, 0};
	pose = arm.setPoint(pos);
	double radius = 0.02;			 // 半径
	double step_angle_degrees = 1.0; // 步进角度
	std::vector<double> pos1 = {0.2, 0, 0.20, -3.1415, 0, 0};
	pose1 = arm.setPoint(pos1);
	std::vector<geometry_msgs::Pose> waypoints;
	generateCirclePoints(pose, radius, step_angle_degrees, waypoints);
	// pose.orientation.w = 1;
	// pose = arm.getCurrent_Pose();
	geometry_msgs::TransformStamped transformStamped;
	tf2::Transform transform;
	try
	{
		// geometry_msgs::TransformStamped transform_base_to_camera;
		// geometry_msgs::TransformStamped transform_base_to_ee;
		// // 获取base_link到camera_link的变换
		// transform_base_to_camera = tfBuffer.lookupTransform("base_link_rm", "camera_link", ros::Time(0));
		// transform_base_to_ee = tfBuffer.lookupTransform("base_link_rm", "ee_link", ros::Time(0));
		// // 反转ee_link到base_link的转换，得到base_link到ee_link的转换
		// tf2::Transform ee_to_base_tf;
		// tf2::fromMsg(transform_base_to_ee.transform, ee_to_base_tf);
		// tf2::Transform base_to_ee_tf = ee_to_base_tf.inverse();

		// // 将camera_link到base_link的转换应用于base_link到ee_link的转换，得到camera_link到ee_link的转换
		// tf2::Transform camera_to_base_tf;
		// tf2::fromMsg(transform_base_to_camera.transform, camera_to_base_tf);
		// tf2::Transform camera_to_ee_tf = base_to_ee_tf * camera_to_base_tf;
		// ROS_INFO("camera_to_ee_tfTranslation: (x: %.2f, y: %.2f, z: %.2f)",
		// 		 camera_to_ee_tf.getOrigin().getX(),
		// 		 camera_to_ee_tf.getOrigin().getY(),
		// 		 camera_to_ee_tf.getOrigin().getZ());
		// ROS_INFO("camera_to_ee_tfRotation: (x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
		// 		 camera_to_ee_tf.getRotation().x(),
		// 		 camera_to_ee_tf.getRotation().y(),
		// 		 camera_to_ee_tf.getRotation().z(),
		// 		 camera_to_ee_tf.getRotation().w());
		transformStamped = tfBuffer.lookupTransform("ee_link", "camera_color_optical_frame",
													ros::Time(0));
		ROS_INFO("Translation: (x: %.2f, y: %.2f, z: %.2f)",
				 transformStamped.transform.translation.x,
				 transformStamped.transform.translation.y,
				 transformStamped.transform.translation.z);
		ROS_INFO("Rotation: (x: %.2f, y: %.2f, z: %.2f, w: %.2f)",
				 transformStamped.transform.rotation.x,
				 transformStamped.transform.rotation.y,
				 transformStamped.transform.rotation.z,
				 transformStamped.transform.rotation.w);
		geometry_msgs::Pose link6_target_pose = calculateTargetTransform(pose, transformStamped.transform);
		// transformStamped.transform.rotation.x=0;
		// transformStamped.transform.rotation.y=0;
		// transformStamped.transform.rotation.z=0;
		// transformStamped.transform.rotation.w=1;
		// tf2::fromMsg(transformStamped.transform, transform);
		// geometry_msgs::Pose trans_pose;
		// trans_pose = arm.transformPose(pose, transform);
		// trans_pose = arm.moveFromPose(pose, -0.1);
		// arm.move_p(trans_pose);
		arm.move_p(pose);
		// arm.move_l(waypoints);
		arm.move_p(link6_target_pose);
	}
	catch (tf2::TransformException &ex)
	{
		ROS_WARN("%s", ex.what());
	}
	// Test
	// arm.transformPose();
	// test for move_j
	// cout<<"-----------------------test for move_j----------------------"<<endl;
	// vector<double> joints = {0, 0, arm.degreesToRadians(-90), 0, arm.degreesToRadians(-90), arm.degreesToRadians(180)};
	// arm.move_j(joints);
	// ros::Duration(5.0).sleep();

	// test for move_p and move_l(1 point)
	// cout<<"-----------------------test for move_p and move_l---------------------"<<endl;
	// vector<double> xyzrpy={0.31,0.067,0.11,0,1,0,0};
	// arm.move_p(xyzrpy);

	// xyzrpy[2]=0.2;
	// arm.move_l(xyzrpy);

	// // test for move_l (>=2 points)
	// cout<<"-----------------------test for move_l(more points)----------------------"<<endl;
	// vector<vector<double>> xyzrpys;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[1]=0.2;
	// xyzrpys.push_back(xyzrpy);
	// xyzrpy[0]=0.4;
	// arm.move_l(xyzrpys);

	// // test for my functions
	// cout<<"-----------------------test for my move_function----------------------"<<endl;
	// double xyz[3]={-0.5,0.02,0.2};
	// arm.move_l(xyz);
	// ros::Duration(5.0).sleep();
	// xyz[2]=0.3;
	// arm.move_l(xyz);
	// ros::Duration(5.0).sleep();
	// vector<double> xyzrpy={0.3,0.1,0.4,-3.1415,0,0};
	// vector<double> xyzrpy1={0.3,0.2,0.3,-3.1415,0,0};
	// arm.move_l({xyzrpy, xyzrpy1});
	// xyz[2]=0.1;
	// arm.move_p_with_constrains(xyz);

	// 控制夹爪
	// arm.Set_Tool_DO(2, true);
	// ros::Duration(2.0).sleep();
	// arm.Set_Tool_DO(2, false);
	// ros::Duration(2.0).sleep();
	// arm.Set_Tool_DO(2, true);

	// // 调用目标检测服务
	// robot_msgs::Hand_Catch srv;
	// srv.request.run = true; // 设置请求标志位
	// if (client.call(srv))
	// {
	// 	ROS_INFO("Service call succeeded");
	// 	// processDetectionResults(srv.response); // 处理检测结果
	// 	const robot_msgs::Hand_CatchResponse &response = srv.response;
	// 	for (size_t i = 0; i < response.labels.size(); ++i)
	// 	{
	// 		std::string label = response.labels[i];
	// 		if (label == "red")
	// 		{
	// 			geometry_msgs::PoseStamped p = response.positions[i];
	// 			std::cout << "Position:" << std::endl;
	// 			std::cout << "x: " << p.pose.position.x << std::endl;
	// 			std::cout << "y: " << p.pose.position.y << std::endl;
	// 			std::cout << "z: " << p.pose.position.z << std::endl;
	// 			std::cout << "Orientation:" << std::endl;
	// 			std::cout << "x: " << p.pose.orientation.x << std::endl;
	// 			std::cout << "y: " << p.pose.orientation.y << std::endl;
	// 			std::cout << "z: " << p.pose.orientation.z << std::endl;
	// 			std::cout << "w: " << p.pose.orientation.w << std::endl;

	// 			// 移动到目标下方
	// 			p.pose.position.z += 0.10;
	// 			arm.move_p(p);
	// 			ROS_INFO("移动到目标上方");
	// 			ros::Duration(2.0).sleep();
	// 			// 打开夹爪
	// 			arm.Set_Tool_DO(2, false);
	// 			ROS_INFO("夹爪开");
	// 			ros::Duration(1.0).sleep();
	// 			// 移动到抓取位置
	// 			p.pose.position.z -= 0.10;
	// 			arm.move_p(p);
	// 			ROS_INFO("摘取目标");
	// 			ros::Duration(1.0).sleep();
	// 			// 关闭夹爪
	// 			arm.Set_Tool_DO(2, true);
	// 			ros::Duration(2.0).sleep();

	// 			vector<double> joints = {0, 0, arm.degreesToRadians(-90), 0, arm.degreesToRadians(-90), arm.degreesToRadians(180)};
	// 			arm.move_j(joints);

	// 			// 打开夹爪
	// 			arm.Set_Tool_DO(2, false);
	// 			ROS_INFO("夹爪开");
	// 			ros::Duration(1.0).sleep();
	// 		}
	// 	}
	// 	// 显示检测结果图像
	// 	cv::Mat detect_image = cv_bridge::toCvCopy(response.detect_image, sensor_msgs::image_encodings::BGR8)->image;
	// 	cv::imshow("Detection Results", detect_image);
	// 	cv::waitKey(0); // 按下任意键继续
	// 	cv::destroyAllWindows();
	// }
	// else
	// {
	// 	ROS_ERROR("Failed to call service objection_detect");
	// }

	ros::waitForShutdown();
	return 0;
}

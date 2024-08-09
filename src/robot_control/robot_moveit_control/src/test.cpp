#include <ros/ros.h>
#include <MoveitServer.h>
#include <robot_msgs/Hand_Catch.h>
using namespace std;

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");																  // 设置编码
	ros::init(argc, argv, "moveit_control_test");										  // 初始化节点
	ros::AsyncSpinner spinner(1);														  // 创建异步spinner，用于处理回调
	ros::NodeHandle nh;																	  // 创建节点句柄
	spinner.start();																	  // 启动异步spinner
	tf2_ros::Buffer tfBuffer;															  // 创建TF缓冲区对象
	tf2_ros::TransformListener tfListener(tfBuffer);									  // 创建TF监听器对象
	std::string PLANNING_GROUP = "rokae_arm";											  // 定义规划组
	MoveitServer arm(PLANNING_GROUP);													  // 创建MoveitServer对象
	// ros::ServiceClient client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect"); // 创建颜色检测服务客户端
																						  // 创建robotTool控件
	robotTool tools;
	// arm.move_p(p);
	arm.move_j(std::vector<double>{tools.degreesToRadians(90), tools.degreesToRadians(-20), tools.degreesToRadians(-60),
								   tools.degreesToRadians(0), tools.degreesToRadians(-100), tools.degreesToRadians(0)});
	// 移动机械臂到检测位置
	geometry_msgs::Pose p = arm.getCurrent_Pose();
	geometry_msgs::Pose p1 = tools.transPose(p, "xMate3_link6","tool");
	tools.publishStaticTFwithRot(p1, "pose1");
	arm.move_l(tools.moveFromPose(p1,0.1));
	arm.move_l(p1);
	arm.move_l(tools.moveFromPose(p1,-0.1));
	// rt.publishStaticTFwithRot(rt.calculateTargetPose(p1,arm.ser))
	// arm.move_p(p1);
	// arm.move_p(p);
	ros::waitForShutdown(); // 等待关闭
	return 0;
}

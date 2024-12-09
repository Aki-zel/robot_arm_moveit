#include <ros/ros.h>
#include <MoveitServer.h>
#include <robot_msgs/Hand_Catch.h>
#include <robotTool.h>
using namespace std;

int main(int argc, char **argv)
{
	setlocale(LC_ALL, "");																  // 设置编码
	ros::init(argc, argv, "example04");													  // 初始化节点
	ros::AsyncSpinner spinner(1);														  // 创建异步spinner，用于处理回调
	ros::NodeHandle nh;																	  // 创建节点句柄
	spinner.start();																	  // 启动异步spinner
	tf2_ros::Buffer tfBuffer;															  // 创建TF缓冲区对象
	tf2_ros::TransformListener tfListener(tfBuffer);									  // 创建TF监听器对象
	std::string PLANNING_GROUP = "arm";													  // 定义规划组
	MoveitServer arm(PLANNING_GROUP);													  // 创建MoveitServer对象
	ros::ServiceClient client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect"); // 创建颜色检测服务客户端
																						  // 创建robotTool控件
	robotTool rt;
	// 移动机械臂到检测位置

	arm.move_j(std::vector<double>{rt.degreesToRadians(0), rt.degreesToRadians(0), rt.degreesToRadians(90),
								   rt.degreesToRadians(0), rt.degreesToRadians(90), rt.degreesToRadians(0)});

	// 调用目标检测服务
	robot_msgs::Hand_Catch srv;
	srv.request.name = "blue"; // 设置目标颜色
	if (client.call(srv))	   // 调用服务
	{
		ROS_INFO("Service call succeeded");

		const robot_msgs::Hand_CatchResponse &response = srv.response; // 获取响应
		std::map<std::string, geometry_msgs::PoseStamped> positions;   // 创建位置映射
		for (size_t i = 0; i < response.labels.size(); ++i)			   // 遍历检测服务检测到的标签
		{
			std::string label = response.labels[i];	  // 获取标签
			positions[label] = response.positions[i]; // 将标签和位置存储到映射中
		}
		double stack_height = 0.0; // 初始化堆叠高度
		// 定义堆叠位置
		vector<double> stack_position;
		stack_position = {0.3, 0.0, 0.0, 0.0, 1, 0.0, 0.0}; // 堆叠位置
		for (size_t i = 0; i < response.labels.size(); ++i)
		{
			std::string label = response.labels[i];		  // 获取标签
			if (positions.find(label) != positions.end()) // 检查标签是否存在
			{
				geometry_msgs::Pose p = positions[label].pose; // 获取标签对应的位置
				// p = rt.transPose(p, "gripper", "Link6");
				p = rt.calculateTargetTransform(p, rt.lookupTransform("Link6", "gripper").transform);
				rt.publishStaticTFwithRot(p, "chess");
				p.position.z += 0.10;
				rt.setGripperPosition(30);
				arm.move_p(p);
				ROS_INFO("移动到目标上方"); // 打印动作信息
				p.position.z -= 0.10;
				arm.move_l(p);
				rt.setGripperPosition(0);
				ROS_INFO("夹取目标");
				p.position.z += 0.10;
				arm.move_p(p);
				rt.setGripperPosition(30);
				ROS_INFO("移动到目标上方");
				// 打印动作信息
				// stack_position[2] = p.position.z + 0.1 + stack_height;
				// arm.move_p(stack_position);
				// ROS_INFO("移动到堆叠位置上方");

				// stack_position[2] -= 0.1;
				// arm.move_p(stack_position);
				// ROS_INFO("放置目标");

				// arm.Set_Tool_DO(2, false);
				// ROS_INFO("夹爪开");

				// stack_height += 0.02; // 更新堆叠高度,根据方块的高度调整

				arm.move_j(std::vector<double>{rt.degreesToRadians(0), rt.degreesToRadians(0), rt.degreesToRadians(90),
											   rt.degreesToRadians(0), rt.degreesToRadians(90), rt.degreesToRadians(0)});
				rt.setGripperPosition(0);
				ROS_INFO("移动回到检测位置");
			}
		}
	}
	ros::waitForShutdown(); // 等待关闭
	return 0;
}

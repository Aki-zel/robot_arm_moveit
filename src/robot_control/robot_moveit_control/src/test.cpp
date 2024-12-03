#include <ros/ros.h>
#include <MoveitServer.h>
#include <robot_msgs/Hand_Catch.h>
#include <csignal> // 用于信号处理
using namespace std;

std::unique_ptr<MoveitServer> moveit_server_ptr; // 用智能指针来管理MoveitServer对象

// 信号处理函数
void signalHandler(int signum)
{
    ROS_INFO("Signal (%d) received, moving arm to home position...", signum);

    if (moveit_server_ptr)
    {
        // moveit_server_ptr->initializeClaw(); // 调用复位函数
    }

    ros::shutdown(); // 安全关闭ROS节点
    exit(signum);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");                        // 设置编码
    ros::init(argc, argv, "moveit_control_test"); // 初始化节点
    ros::AsyncSpinner spinner(1);                 // 创建异步spinner，用于处理回调
    ros::NodeHandle nh;                           // 创建节点句柄
    spinner.start();                              // 启动异步spinner

    tf2_ros::Buffer tfBuffer;                        // 创建TF缓冲区对象
    tf2_ros::TransformListener tfListener(tfBuffer); // 创建TF监听器对象

    std::string PLANNING_GROUP = "arm";                                 // 定义规划组
    moveit_server_ptr = std::make_unique<MoveitServer>(PLANNING_GROUP); // 创建MoveitServer对象
    robotTool tools;
    tools.setModusMod();
    ros::Duration(1).sleep();
    // tools.publishCommand(1,1,std::vector<uint16_t>{0,100});
    tools.setGripperPosition(100);
    ros::Duration(3).sleep();
    tools.setGripperPosition(50);
    ros::Duration(3).sleep();
    tools.setGripperPosition(0);
    // 注册信号处理函数
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    ros::waitForShutdown(); // 等待节点关闭
    return 0;
}

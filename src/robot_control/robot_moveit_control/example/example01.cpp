#include <ros/ros.h>
#include <MoveitServer.h>

int main(int argc, char *argv[])
{
    // 支持终端输出中文
    setlocale(LC_ALL, "");
    ROS_INFO("中文输出！！！");
    // ros初始化
    ros::init(argc, argv, "example01");
    ros::NodeHandle nh;
    ROS_INFO("ros 创建成功！！！");
    // 创建ros 多线程
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_INFO("支持多线程！！！");
    // 创建moveitserver控件
    std::string PLANNING_GROUP = "arm";
    MoveitServer arm(PLANNING_GROUP);
    ROS_INFO("创建moveitserver控件完成");
    // 使用move_j控制机械臂到某姿态,数据为弧度
    std::vector<double> j = {0.15, 1, 0, 0, 1.5, 1.5};
    arm.move_j(j);
    ROS_INFO("运行move joint");
    auto current_pose = arm.getCurrent_Pose();
    ROS_INFO("获取当前的坐标点");
    ROS_INFO("位置信息: x: %f, y: %f, z: %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    ROS_INFO("方向信息: qx: %f, qy: %f, qz: %f, qw: %f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    // 使用move_p控制机械臂到某一姿态
    geometry_msgs::Pose p;
    p.position.x = 0.2;
    p.position.y = 0;
    p.position.z = 0.2;
    p.orientation.w = 1;
    // 也可用这种方式创建目标
    // std::vector<double> p={0.2,0,0.2,0,0,0,1};
    arm.move_p(p);
    ROS_INFO("运行move position");

    std::vector<double> current_joint;
    arm.getCurrentJoint(current_joint);
    ROS_INFO("获取当前关节角度");
    int i = 0;
    for (auto joint : current_joint)
    {
        ROS_INFO("Joint %d :%f", i, joint);
    }
    arm.setMaxVelocity(0.4);
    ROS_INFO("设置最大速度为40%%");
    // 使用go_home回到起点
    arm.go_home();
    ROS_INFO("回到起点");

    ros::waitForShutdown(); 
    return 0;
}

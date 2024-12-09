#include <ros/ros.h>
#include <MoveitServer.h>
#include <robotTool.h>
int main(int argc, char *argv[])
{
    // 支持终端输出中文
    setlocale(LC_ALL, "");
    ROS_INFO("中文输出！！！");
    // ros初始化
    ros::init(argc, argv, "example02");
    ros::NodeHandle nh;
    ROS_INFO("ros 创建成功！！！");
    // 创建ros 多线程
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_INFO("支持多线程！！！");
    // 创建moveitserver控件
    std::string PLANNING_GROUP = "arm";
    MoveitServer arm(PLANNING_GROUP);
    ROS_INFO("创建MoveitServer控件完成");
    // 创建robotTool控件
    robotTool rt;
    ROS_INFO("创建robotTool控件完成");
    // 使用move_j控制机械臂到某姿态,通过rt直观设置需要的角度
    std::vector<double> j = {rt.degreesToRadians(-130), rt.degreesToRadians(2), rt.degreesToRadians(92),
                             rt.degreesToRadians(0), rt.degreesToRadians(83), rt.degreesToRadians(-121)};
    arm.move_j(j);
    ROS_INFO("运行move joint");
    auto current_pose = arm.getCurrent_Pose();
    rt.setGripperPosition(30);
    ROS_INFO("获取当前的坐标点");
    ROS_INFO("位置信息: x: %f, y: %f, z: %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
    ROS_INFO("方向信息: qx: %f, qy: %f, qz: %f, qw: %f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
    // 使用rt当前z轴向下0.1；
    auto p = rt.moveFromPose(current_pose, 0.1);
    auto p1 = rt.moveFromPose(current_pose, 0.095);
    //  通过rt打印坐标点展示在rviz上
    rt.publishStaticTFwithRot(current_pose, "current");
    rt.publishStaticTFwithRot(p, "new pose");
    rt.setGripperForce(100);
    while (ros::ok())
    {
        arm.move_l(p);
        rt.setGripperPosition(0);
        ros::Duration(1).sleep();
        ROS_INFO("运行move position");
        arm.move_l(current_pose);
        arm.move_l(p1);
        rt.setGripperPosition(30);
        arm.move_l(current_pose);
    }
    // 使用go_home回到起点
    // arm.go_home();
    ROS_INFO("回到起点");

    ros::waitForShutdown();
    return 0;
}

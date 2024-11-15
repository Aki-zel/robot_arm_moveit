#include <ros/ros.h>
#include <MoveitServer.h>
#include <robotTool.h>
#include <robot_msgs/Hand_Catch.h>

int main(int argc, char *argv[])
{
    // 支持终端输出中文
    setlocale(LC_ALL, "");
    // ros初始化
    ros::init(argc, argv, "example03");
    ros::NodeHandle nh;
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
    ros::ServiceClient object_client, color_client;
    object_client = nh.serviceClient<robot_msgs::Hand_Catch>("object_detect");
    // color_client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
    ROS_INFO("创建视觉识别控件");
    // 移动到识别预选位置
	// arm.move_j(std::vector<double>{rt.degreesToRadians(0), rt.degreesToRadians(0), rt.degreesToRadians(90),
	// 										 rt.degreesToRadians(0), rt.degreesToRadians(90), rt.degreesToRadians(0)});
    ROS_INFO("移动到识别预选位置");
    robot_msgs::Hand_Catch run;
    run.request.run = true;
    // 需要抓取物品的名称
    run.request.name = "can";
    ROS_INFO("调用视觉识别服务");
    auto current_pose = arm.getCurrent_Pose();
    ros::Duration(1).sleep();
    if (object_client.call(run))
    {
        bool success = true;
        int i = 0;
        ROS_INFO("简单的物品抓取");
        for (auto p : run.response.positions)
        {
            auto pose = p.pose;
            ROS_INFO("物品名称：%s", run.response.labels[i++].c_str());
            pose=rt.moveFromPose(pose, -0.145);
            ROS_INFO("位置信息: x: %f, y: %f, z: %f", pose.position.x, pose.position.y, pose.position.z);
            ROS_INFO("方向信息: qx: %f, qy: %f, qz: %f, qw: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            success = arm.move_p(rt.moveFromPose(pose, -0.10), success);
            arm.Set_Tool_DO(2, false);
            success = arm.move_l(pose, success);
            arm.Set_Tool_DO(2, true);
            success = arm.move_l(rt.moveFromPose(pose, -0.10), success);
            success = arm.move_p(current_pose);
            arm.Set_Tool_DO(2, false);
        }
    }
    ros::waitForShutdown();
    return 0;
}

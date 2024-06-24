#include <bits/stdc++.h>
#include <ros/ros.h>
#include <robot_msgs/Hand_Catch.h>
#include <robot_msgs/Task_Call.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <vgn/GetSceneCloud.h>
#include <vgn/GetMapCloud.h>
#include <vgn/PredictGrasps.h>
#include <vgn/GraspConfig.h>
#include <MoveitServer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
ros::Subscriber a;
ros::Publisher iswork, current_state;
ros::ServiceClient object_client, color_client, resetmap, toggle, get_scene_cloud_client, get_map_cloud_client, predict_grasps_client;
ros::ServiceServer CallTask;

void resetMap()
{
    std_srvs::Empty em;
    resetmap.call(em);
}
void setToggle(bool value)
{
    std_srvs::SetBool toggle_integration_srv;
    toggle_integration_srv.request.data = value;
    toggle.call(toggle_integration_srv);
}
vgn::GetMapCloud getSense()
{
    vgn::GetSceneCloud get_scene_cloud_srv;
    vgn::GetMapCloud get_map_cloud_srv;
    get_scene_cloud_client.call(get_scene_cloud_srv);
    get_map_cloud_client.call(get_map_cloud_srv);
    return get_map_cloud_srv;
}
void preditGrasp(vgn::GetMapCloud sensemap)
{
    vgn::PredictGrasps predict_grasps_srv;
    predict_grasps_srv.request.map_cloud = sensemap.response.map_cloud;
    predict_grasps_srv.request.voxel_size = sensemap.response.voxel_size;
    predict_grasps_client.call(predict_grasps_srv);
    predict_grasps_srv.response.grasps.data();
}
void publishWorkStatus(const ros::TimerEvent &, ros::Publisher &iswork_pub)
{
    std_msgs::String msg;
    msg.data = "on work";
    iswork_pub.publish(msg);
}
// 发布静态TF的函数
void publishStaticTF(const geometry_msgs::Pose &p)
{
    static tf2_ros::StaticTransformBroadcaster broadcaster;
    // ROS_INFO("Publishing static TF named task");

    // 创建坐标系信息
    geometry_msgs::TransformStamped ts;
    // 设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link_rm";
    // 设置子级坐标系
    ts.child_frame_id = "task";
    // 设置子级相对于父级的偏移量
    ts.transform.translation.x = p.position.x;
    ts.transform.translation.y = p.position.y;
    ts.transform.translation.z = p.position.z;
    // 设置四元数: 将欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, 0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // 广播器发布坐标系信息
    broadcaster.sendTransform(ts);
}

void timerCallback(const ros::TimerEvent &, const geometry_msgs::Pose &p)
{
    publishStaticTF(p);
}

bool openDoor(MoveitServer &arm)
{
    bool success;
    ROS_INFO_NAMED("openDoor", "openDoor");
    try
    {
        ROS_INFO_NAMED("openDoor", "移动到识别点");
        success = arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), -arm.degreesToRadians(50.00),
                                                 0, -arm.degreesToRadians(90), arm.degreesToRadians(180)});
        geometry_msgs::Pose target;
        robot_msgs::Hand_Catch run;
        run.request.run = true;
        if (object_client.call(run) && !run.response.positions.empty())
        {
            target = run.response.positions.front().pose;
            // target = arm.setPoint(std::vector<double>{0.257, 0, 0.6637, 0, arm.degreesToRadians(90), 0});
            success = arm.move_p(target, success);
            geometry_msgs::Pose target1;
            target1 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{-0.05, 0, 0, 0, 0, arm.degreesToRadians(30)}));
            success = arm.move_l(target1, success);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.10, 0, 0, arm.degreesToRadians(0)}));
            success = arm.move_l(target1, success);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.2, 0, 0, arm.degreesToRadians(-30)}));
            success = arm.move_l(target1, success);
            ROS_INFO_NAMED("openDoor", "回到识别点");
            success = arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), -arm.degreesToRadians(50.00),
                                                     0, -arm.degreesToRadians(90), arm.degreesToRadians(180)},
                                 success);
        }
        if (success == false)
        {
            arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), -arm.degreesToRadians(50.00),
                                           0, -arm.degreesToRadians(90), arm.degreesToRadians(180)});
            return success;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return success;
};
bool openCabinet(MoveitServer &arm)
{
    bool success;
    ROS_INFO_NAMED("openCabinet", "openCabinet");
    ros::NodeHandle nh;
    try
    {
        ROS_INFO_NAMED("openCabinet", "移动到识别点");
        success = arm.move_j(std::vector<double>{arm.degreesToRadians(28), arm.degreesToRadians(-52), arm.degreesToRadians(-99),
                                                 arm.degreesToRadians(124), arm.degreesToRadians(89), arm.degreesToRadians(150)});
        geometry_msgs::Pose target;
        robot_msgs::Hand_Catch run;
        run.request.run = true;
        // object_client.call(run);
        if (color_client.call(run) && !run.response.positions.empty())
        {
            target = run.response.positions[0].pose;
            target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, arm.degreesToRadians(90), arm.degreesToRadians(90), 0});
            // success = arm.move_p(target, success);
            arm.Set_Tool_DO(2, false);
            geometry_msgs::Pose target1;
            ROS_INFO_NAMED("openCabinet", "1");
            target1 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, arm.degreesToRadians(90)}));
            success = arm.move_p(target1, success);
            ROS_INFO_NAMED("openCabinet", "2");
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, -0.005, -0.11, 0, 0, 0}));
            success = arm.move_l(target1, success);
            arm.Set_Tool_DO(2, true);
            ROS_INFO_NAMED("openCabinet", "3");
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.23, 0, 0, 0}));
            success = arm.move_l(target1, success);
            arm.Set_Tool_DO(2, false);
            ros::Duration(1).sleep();
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.05, 0, 0, 0}));
            success = arm.move_l(target1, success);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0.17, 0, 0, 0, 0, arm.degreesToRadians(-90)}));
            success = arm.move_l(target1, success);
            if (color_client.call(run) && !run.response.positions.empty())
            {
                target = run.response.positions[0].pose;
                target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, arm.degreesToRadians(90), arm.degreesToRadians(90), 0});
                target1 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, arm.degreesToRadians(90)}));
                success = arm.move_l(target1, success);
                target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, -0.005, -0.11, 0, 0, 0}));
                success = arm.move_l(target1, success);
                target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.23, 0, 0, 0}));
                success = arm.move_l(target1, success);
                target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.05, 0, 0, 0}));
                success = arm.move_l(target1, success);
                target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0.1, 0, 0, 0, 0, arm.degreesToRadians(-90)}));
                success = arm.move_l(target1, success);
            }
            // publishStaticTF(target1);
            ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timerCallback, _1, target1));
            // ROS_INFO_NAMED("openCabinet", "回到识别点");
            // success = arm.move_j(std::vector<double>{arm.degreesToRadians(30), arm.degreesToRadians(-52), arm.degreesToRadians(-98),
            //                                          arm.degreesToRadians(126), arm.degreesToRadians(92), arm.degreesToRadians(152)},
            //                      success);
        }
        if (success == false)
        {
            success = arm.move_j(std::vector<double>{arm.degreesToRadians(28), arm.degreesToRadians(-52), arm.degreesToRadians(-99),
                                                     arm.degreesToRadians(124), arm.degreesToRadians(89), arm.degreesToRadians(150)});
            return success;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return success;
}
bool targetCapture(MoveitServer &arm)
{
    bool success;
    ROS_INFO_NAMED("targetCapture", "targetCapture");
    try
    {
        ROS_INFO_NAMED("targetCapture", "移动到识别点");
        arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), arm.degreesToRadians(-50.00),
                                       0, arm.degreesToRadians(-90), arm.degreesToRadians(180)});
        geometry_msgs::Pose target;
        robot_msgs::Hand_Catch run;
        run.request.run = true;
        // object_client.call(run);
        if (object_client.call(run) && !run.response.positions.empty())
        {
            target = run.response.positions[0].pose;
            // target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, 0, arm.degreesToRadians(90), 0});
            arm.Set_Tool_DO(2, false);
            resetMap();
        }
        if (success == false)
        {
            arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), arm.degreesToRadians(-50.00),
                                           0, arm.degreesToRadians(-90), arm.degreesToRadians(180)});
            return success;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }

    return success;
}
bool callTask(robot_msgs::Task_Call::Request &req,
              robot_msgs::Task_Call::Response &rep,
              MoveitServer &arm)
{
    switch (req.taskId)
    {
    case 1:
        ROS_INFO_NAMED("TASK", "Start taskId 1 named 'open door'. ");
        rep.succeed = openDoor(arm);
        break;
    case 2:
        ROS_INFO_NAMED("TASK", "Start taskId 2 named 'open cabinet'. ");
        rep.succeed = openCabinet(arm);
        break;
    case 3:
        ROS_INFO_NAMED("TASK", "Start taskId 3 named 'target capture'. ");
        rep.succeed = targetCapture(arm);
        break;
    default:
        break;
    }
    rep.succeed = true;
    return true;
}
void initialize_service_client(ros::NodeHandle nh)
{
    object_client = nh.serviceClient<robot_msgs::Hand_Catch>("objection_detect");
    color_client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
    resetmap = nh.serviceClient<std_srvs::Empty>("reset_map");
    toggle = nh.serviceClient<std_srvs::SetBool>("toggle_integration");
    get_scene_cloud_client = nh.serviceClient<vgn::GetSceneCloud>("get_scene_cloud");
    get_map_cloud_client = nh.serviceClient<vgn::GetMapCloud>("get_map_cloud");
    predict_grasps_client = nh.serviceClient<vgn::PredictGrasps>("predict_grasps");
    ros::service::waitForService("color_detect");
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ArmControlCenter");
    ros::NodeHandle nh;
    initialize_service_client(nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::string PLANNING_GROUP = "arm";
    MoveitServer arm(PLANNING_GROUP);
    CallTask = nh.advertiseService<robot_msgs::Task_Call::Request, robot_msgs::Task_Call::Response>(
        "call_task",
        std::bind(&callTask, std::placeholders::_1, std::placeholders::_2, std::ref(arm)));
    iswork = nh.advertise<std_msgs::String>("iswork", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(publishWorkStatus, _1, boost::ref(iswork)));
    ros::waitForShutdown();
    return 0;
}

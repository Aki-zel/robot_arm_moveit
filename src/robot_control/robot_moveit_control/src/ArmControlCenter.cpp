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
bool openDoor(MoveitServer &arm)
{
    bool success;
    try
    {
        ROS_INFO_NAMED("OPENDOOR", "移动到识别点");
        success = arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), -arm.degreesToRadians(50.00),
                                                 0, -arm.degreesToRadians(90), arm.degreesToRadians(180)});
        geometry_msgs::Pose target;
        target = arm.setPoint(std::vector<double>{0.257, 0, 0.6637, 0, arm.degreesToRadians(90), 0});
        success = arm.move_l(target, success);
        // success = arm.move_p(std::vector<double>{0.357, 0, 0.6237, 0, arm.degreesToRadians(90), arm.degreesToRadians(30)});
        geometry_msgs::Pose target1;
        target1 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{-0.05, 0, 0, 0, 0, arm.degreesToRadians(30)}));
        success = arm.move_l(target1, success);
        // success = arm.move_p(std::vector<double>{0.357, 0, 0.6237, 0, arm.degreesToRadians(90), arm.degreesToRadians(30)});
        target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.10, 0, 0, arm.degreesToRadians(-30)}));
        success = arm.move_l(target1, success);
        target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.2, 0, 0, arm.degreesToRadians(0)}));
        success = arm.move_l(target1, success);
        ROS_INFO_NAMED("OPENDOOR", "回到识别点");
        success = arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), -arm.degreesToRadians(50.00),
                                                 0, -arm.degreesToRadians(90), arm.degreesToRadians(180)},
                             success);
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

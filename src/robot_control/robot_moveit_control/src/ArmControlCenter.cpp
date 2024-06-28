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
geometry_msgs::Pose transP(geometry_msgs::Pose p)
{
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::PoseStamped pose_stamped_in;
    pose_stamped_in.pose = p;
    pose_stamped_in.header.frame_id = "task";
    pose_stamped_in.header.stamp = ros::Time(0); // 使用最新的变换

    geometry_msgs::PoseStamped pose_stamped_out;
    try
    {
        // 等待变换可用
        tfBuffer.canTransform("base_link_rm", "task", ros::Time(0), ros::Duration(3.0));
        // 获取变换，并将pose从task变换到base_link_rm
        tfBuffer.transform(pose_stamped_in, pose_stamped_out, "base_link_rm");
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform warning: %s", ex.what());
    }
    return pose_stamped_out.pose;
}
void publishGraspPoses(const std::vector<vgn::GraspConfig> &grasps)
{
    static tf2_ros::TransformBroadcaster broadcaster;
    for (size_t i = 0; i < grasps.size(); ++i)
    {
        geometry_msgs::Pose transformed_pose = transP(grasps[i].pose);
        geometry_msgs::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros::Time::now();
        transform_stamped.header.frame_id = "base_link_rm";
        transform_stamped.child_frame_id = "grasp_" + std::to_string(i);
        ROS_INFO("%s", transform_stamped.child_frame_id.c_str());
        transform_stamped.transform.translation.x = transformed_pose.position.x;
        transform_stamped.transform.translation.y = transformed_pose.position.y;
        transform_stamped.transform.translation.z = transformed_pose.position.z;

        transform_stamped.transform.rotation.x = transformed_pose.orientation.x;
        transform_stamped.transform.rotation.y = transformed_pose.orientation.y;
        transform_stamped.transform.rotation.z = transformed_pose.orientation.z;
        transform_stamped.transform.rotation.w = transformed_pose.orientation.w;

        broadcaster.sendTransform(transform_stamped);
    }
}
void preditGrasp(vgn::GetMapCloud sensemap, std::vector<geometry_msgs::Pose> &ret)
{
    vgn::PredictGrasps predict_grasps_srv;
    predict_grasps_srv.request.map_cloud = sensemap.response.map_cloud;
    predict_grasps_srv.request.voxel_size = sensemap.response.voxel_size;
    if (predict_grasps_client.call(predict_grasps_srv))
    {
        const std::vector<vgn::GraspConfig> &grasps = predict_grasps_srv.response.grasps;
        if (!grasps.empty())
        {
            const std::vector<vgn::GraspConfig> &grasps = predict_grasps_srv.response.grasps;
            std::vector<vgn::GraspConfig> valid_grasps; // 用于存放符合条件的姿态

            for (const auto &grasp : grasps)
            {
                tf2::Quaternion quat;
                tf2::fromMsg(grasp.pose.orientation, quat);

                // 定义Z轴单位向量
                tf2::Vector3 z_axis(0, 0, 1);

                // 将四元数应用于Z轴向量，以获得在基础坐标系中的方向向量
                tf2::Vector3 transformed_z_axis = tf2::quatRotate(quat, z_axis);

                // 现在可以检查transformed_z_axis的z分量来确定Z轴方向
                if (transformed_z_axis.z() <= 0.0)
                {
                    double angle = acos(transformed_z_axis.dot(tf2::Vector3(0, 0, -1)));
                    // 将夹角转换为度数
                    double angle_deg = angle * 180.0 / M_PI;
                    if (angle_deg <= 30.0)
                    {
                        geometry_msgs::Pose transformed_pose = transP(grasp.pose);
                        ret.push_back(transformed_pose);
                        valid_grasps.push_back(grasp); // 如果夹角在-45到45度之间，将该姿态加入有效姿态列表
                    }
                }
            }
            publishGraspPoses(valid_grasps);
        }
        else
        {
            ROS_ERROR("Failed to call service PredictGrasps");
        }
    }
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
        // 此段代码的相对位置为 x->z ,y->x,z->y
        ROS_INFO_NAMED("openCabinet", "移动到识别点");
        success = arm.move_j(std::vector<double>{arm.degreesToRadians(46), arm.degreesToRadians(-86), arm.degreesToRadians(-62),
                                                 arm.degreesToRadians(132), arm.degreesToRadians(112), arm.degreesToRadians(155)});
        geometry_msgs::Pose target, target1, target2, target3;
        robot_msgs::Hand_Catch run;
        run.request.run = true;
        run.request.color_name = "cabinet_handle";
        // object_client.call(run);
        if (color_client.call(run) && !run.response.positions.empty())
        {
            target = run.response.positions[1].pose;
            target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, arm.degreesToRadians(90), arm.degreesToRadians(90), 0});
            // success = arm.move_p(target, success);
            arm.Set_Tool_DO(2, false);
            ROS_INFO_NAMED("openCabinet", "1");
            target1 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, arm.degreesToRadians(90)}));
            success = arm.move_p(target1, success);
            ROS_INFO_NAMED("openCabinet", "2");
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.11, 0, 0, 0}));
            success = arm.move_l(target1, success);
            arm.Set_Tool_DO(2, true);
            ROS_INFO_NAMED("openCabinet", "3");
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, -0.01, 0.23, 0, 0, 0}));
            success = arm.move_l(target1, success);
            arm.Set_Tool_DO(2, false);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.05, 0, 0, 0}));
            success = arm.move_l(target1, success);
            target3 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0.17, 0, 0, 0, 0, arm.degreesToRadians(-90)}));
            success = arm.move_l(target3, success);
            // ROS_INFO_NAMED("openCabinet", "4");
            // target = run.response.positions[0].pose;s
            // target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, arm.degreesToRadians(90), arm.degreesToRadians(90), 0});
            // target2 = arm.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, arm.degreesToRadians(90)}));
            // success = arm.move_l(target2, success);
            // target2 = arm.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, -0.11, 0, 0, 0}));
            // success = arm.move_l(target2, success);
            // arm.Set_Tool_DO(2, true);
            // target2 = arm.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, 0.23, 0, 0, 0}));
            // success = arm.move_l(target2, success);
            // arm.Set_Tool_DO(2, false);
            // target2 = arm.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, 0.05, 0, 0, 0}));
            // success = arm.move_l(target2, success);
            // target3 = arm.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0.15, 0, 0, 0, 0, arm.degreesToRadians(-90)}));
            // success = arm.move_l(target3, success);
            // target3 = arm.setPoint(std::vector<double>{target.position.x - 0.10, target.position.y - 0.02, target.position.z - 0.04, 0, 0, 0});
            // publishStaticTF(target1);
            // ROS_INFO("Publish TF");
            // ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timerCallback, _1, target3));

            success = arm.move_j(std::vector<double>{arm.degreesToRadians(-43), arm.degreesToRadians(6), arm.degreesToRadians(-28),
                                                     arm.degreesToRadians(159), arm.degreesToRadians(106), arm.degreesToRadians(3)},
                                 success);
            // 移动到正面朝向
            target3 = arm.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0, 0, 0, arm.degreesToRadians(-90), arm.degreesToRadians(90)}));
            success = arm.move_p(target3, success);
            target3 = arm.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0, 0, 0, 0, arm.degreesToRadians(-90)}));
            success = arm.move_p(target3, success);
            target3 = arm.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0.2, -0.05, 0, 0, 0}));
            success = arm.move_l(target3, success);
            arm.Set_Tool_DO(2, false);
            target3 = arm.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, -0.2, 0.05, 0, 0, 0}));
            success = arm.move_l(target3, success);
            success = arm.move_j(std::vector<double>{arm.degreesToRadians(-43), arm.degreesToRadians(6), arm.degreesToRadians(-28),
                                                     arm.degreesToRadians(159), arm.degreesToRadians(106), arm.degreesToRadians(3)},
                                 success);
            success = arm.move_p(target1, success);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.28, 0, 0, 0}));
            success = arm.move_l(target1, success);
            target1 = arm.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.28, 0, 0, 0}));
            success = arm.move_l(target1, success);
        }
        if (success == false)
        {
            arm.move_j(std::vector<double>{arm.degreesToRadians(46), arm.degreesToRadians(-86), arm.degreesToRadians(-62),
                                           arm.degreesToRadians(132), arm.degreesToRadians(112), arm.degreesToRadians(155)});
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
    bool success = true;
    ROS_INFO_NAMED("targetCapture", "targetCapture");
    try
    {
        // ROS_INFO_NAMED("targetCapture", "移动到识别点");
        // arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), arm.degreesToRadians(-50.00),
        //                                0, arm.degreesToRadians(-90), arm.degreesToRadians(180)});
        geometry_msgs::Pose target, target3;
        robot_msgs::Hand_Catch run;
        std::vector<geometry_msgs::Pose> ps;
        run.request.run = true;
        run.request.color_name = "blue";
        arm.setMaxVelocity(0.2);
        // object_client.call(run);
        if (color_client.call(run) && !run.response.positions.empty())
        {
            for (auto p : run.response.positions)
            {
                // target = run.response.positions[0].pose;
                // success = arm.move_p(run.response.positions[0].pose);
                target = arm.setPoint(std::vector<double>{p.pose.position.x, p.pose.position.y, p.pose.position.z, 0, 0, 0});
                target3 = arm.setPoint(std::vector<double>{target.position.x - 0.04, target.position.y - 0.04, target.position.z - 0.03, 0, 0, 0});
                publishStaticTF(target3);
                ROS_INFO("Publish TF");
                arm.Set_Tool_DO(2, false);
                resetMap();
                setToggle(true);
                ros::Duration(0.5).sleep();
                setToggle(false);
                preditGrasp(getSense(), ps);
                for (auto pose : ps)
                {
                    geometry_msgs::Pose pose1;
                    success = arm.move_p(pose);
                    pose1 = arm.calculateTargetPose(pose, arm.setPoint(std::vector<double>{0, 0, -0.02, 0, 0, 0}));
                    success = arm.move_p(pose1, success);
                    arm.Set_Tool_DO(2, true);
                    success = arm.move_l(pose, success);
                    // arm.move_j(std::vector<double>{arm.degreesToRadians(-41), arm.degreesToRadians(33), arm.degreesToRadians(-50),
                    //                                arm.degreesToRadians(169), arm.degreesToRadians(124), arm.degreesToRadians(3)});
                    // arm.Set_Tool_DO(2, false);
                    if (success)
                        break;
                }
                ps.clear();
            }

            // while (!ps.empty())
            // {
            //     for (auto pose : ps)
            //     {
            //         geometry_msgs::Pose pose1;
            //         success = arm.move_p(pose);
            //         pose1 = arm.moveFromPose(pose, 0.03);
            //         success = arm.move_l(pose1, success);
            //         arm.Set_Tool_DO(2, true);
            //         pose1 = arm.moveFromPose(pose, -0.08);
            //         success = arm.move_l(pose1, success);
            //         arm.move_j(std::vector<double>{arm.degreesToRadians(-41), arm.degreesToRadians(33), arm.degreesToRadians(-50),
            //                                        arm.degreesToRadians(169), arm.degreesToRadians(124), arm.degreesToRadians(3)});
            //         arm.Set_Tool_DO(2, false);
            //         if (success)
            //             break;
            //     }
            //     ps.clear();
            //     resetMap();
            //     setToggle(true);
            //     ros::Duration(0.5).sleep();
            //     setToggle(false);
            //     preditGrasp(getSense(), ps);
            // }
        }
        if (success == false)
        {
            // arm.move_j(std::vector<double>{0, arm.degreesToRadians(50.00), arm.degreesToRadians(-50.00),
            //                                0, arm.degreesToRadians(-90), arm.degreesToRadians(180)});
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

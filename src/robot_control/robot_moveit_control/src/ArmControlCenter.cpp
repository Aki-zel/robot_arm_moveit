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
#include <yd_msgs/Pose_Task.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/Call_TaskAction.h>
#include <robot_msgs/Objection_Detect.h>
#include <robotTool.h>

typedef actionlib::SimpleActionServer<robot_msgs::Call_TaskAction> Server;

// moveit_visual_tools::MoveItVisualTools *visual_tools ;
class ControlCenter
{
public:
    ControlCenter(ros::NodeHandle &nh, std::string PLANNING_GROUP)
        : nh_(nh), arm(PLANNING_GROUP)
    {

        initialize_service_client(nh_);
        iswork = nh_.advertise<std_msgs::String>("iswork", 1);
        timer = nh_.createTimer(ros::Duration(0.5), boost::bind(&ControlCenter::publishWorkStatus, this, _1, boost::ref(iswork)));
        CallTask = nh_.advertiseService<robot_msgs::Task_Call::Request, robot_msgs::Task_Call::Response>(
            "call_task",
            std::bind(&ControlCenter::callTask, this, std::placeholders::_1, std::placeholders::_2));
        server = new Server(nh_, "call_task_action", boost::bind(&ControlCenter::callTaskAction, this, _1), false);
        server->start();
        ROS_INFO_NAMED("TASK", "Start Call Task server. ");

        // object_client_realtime.waitForServer();
        // arm.setCollisionMatrix();
    }
    void initialize_service_client(ros::NodeHandle nh)
    {

        object_client = nh.serviceClient<robot_msgs::Hand_Catch>("object_detect");
        color_client = nh.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        resetmap = nh.serviceClient<std_srvs::Empty>("reset_map");
        toggle = nh.serviceClient<std_srvs::SetBool>("toggle_integration");
        get_scene_cloud_client = nh.serviceClient<vgn::GetSceneCloud>("get_scene_cloud");
        get_map_cloud_client = nh.serviceClient<vgn::GetMapCloud>("get_map_cloud");
        predict_grasps_client = nh.serviceClient<vgn::PredictGrasps>("predict_grasps");
        carControl = nh.serviceClient<yd_msgs::Pose_Task>("/Rm_TargetPose");
        object_client_realtime = nh.serviceClient<robot_msgs::Objection_Detect>("object_realtime_detect");
        ROS_INFO_NAMED("TASK", "initialize_service_client. ");
    }

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
                        if (angle_deg <= 50.0)
                        {
                            geometry_msgs::Pose transformed_pose = tool.transPose(grasp.pose);
                            ret.push_back(transformed_pose);
                            valid_grasps.push_back(grasp); // 如果夹角在-45到45度之间，将该姿态加入有效姿态列表
                        }
                    }
                }
                tool.publishGraspPoses(valid_grasps);
            }
            else
            {
                ROS_ERROR("Failed to call service PredictGrasps");
            }
        }
    }
    void publishWorkStatus(const ros::TimerEvent &, ros::Publisher &iswork_pub)
    {
        statemsg.data = "on work";
        iswork_pub.publish(statemsg);
    }

    void timerCallback(const ros::TimerEvent &, const geometry_msgs::Pose &p)
    {
        tool.publishStaticTF(p);
    }
    bool openDoor()
    {
        bool success;
        ROS_INFO_NAMED("openDoor", "openDoor");
        try
        {
            ROS_INFO_NAMED("openDoor", "移动到识别点");
            success = arm.move_j(std::vector<double>{0, tool.degreesToRadians(50.00), -tool.degreesToRadians(50.00),
                                                     0, -tool.degreesToRadians(90), tool.degreesToRadians(180)});
            geometry_msgs::Pose target;
            robot_msgs::Hand_Catch run;
            run.request.run = true;
            if (object_client.call(run) && !run.response.positions.empty())
            {
                target = run.response.positions.front().pose;
                // target = arm.setPoint(std::vector<double>{0.257, 0, 0.6637, 0, tool.degreesToRadians(90), 0});
                success = arm.move_p(target, success);
                geometry_msgs::Pose target1;
                target1 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0.05, 0, 0, 0, 0, tool.degreesToRadians(-30)}));
                success = arm.move_l(target1, success);
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.10, 0, 0, tool.degreesToRadians(0)}));
                success = arm.move_l(target1, success);
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.2, 0, 0, tool.degreesToRadians(30)}));
                success = arm.move_l(target1, success);
                ROS_INFO_NAMED("openDoor", "回到识别点");
                success = arm.move_j(std::vector<double>{0, tool.degreesToRadians(50.00), -tool.degreesToRadians(50.00),
                                                         0, -tool.degreesToRadians(90), tool.degreesToRadians(180)},
                                     success);
            }
            if (success == false)
            {
                arm.move_j(std::vector<double>{0, tool.degreesToRadians(50.00), -tool.degreesToRadians(50.00),
                                               0, -tool.degreesToRadians(90), tool.degreesToRadians(180)});
                return success;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return success;
    };
    bool colorCapture()
    {
        bool success = true;
        ROS_INFO_NAMED("colorCapture", "colorCapture");
        try
        {
            // ROS_INFO_NAMED("targetCapture", "移动到识别点");
            // arm.move_j(std::vector<double>{0, tool.degreesToRadians(50.00), tool.degreesToRadians(-50.00),
            //                                0, tool.degreesToRadians(-90), tool.degreesToRadians(180)});
            geometry_msgs::Pose target, target3, start_pose;
            robot_msgs::Hand_Catch run;
            std::vector<geometry_msgs::Pose> ps;
            run.request.run = true;
            run.request.name = "blue";
            arm.setMaxVelocity(0.2);
            // object_client.call(run);
            start_pose = arm.getCurrent_Pose();
            if (color_client.call(run) && !run.response.positions.empty())
            {
                for (auto p : run.response.positions)
                {
                    // target = run.response.positions[0].pose;
                    // success = arm.move_p(run.response.positions[0].pose);
                    target = arm.setPoint(std::vector<double>{p.pose.position.x, p.pose.position.y, p.pose.position.z, 0, 0, 0});
                    target3 = arm.setPoint(std::vector<double>{target.position.x - 0.04, target.position.y - 0.04, target.position.z - 0.03, 0, 0, 0});
                    tool.publishStaticTF(target3);
                    arm.Set_Tool_DO(2, false);
                    // resetMap();
                    // setToggle(true);
                    // ros::Duration(0.5).sleep();
                    // setToggle(false);
                    // preditGrasp(getSense(), ps);
                    // for (auto pose : ps)
                    // {

                    geometry_msgs::Pose pose1;
                    //  tool.publishStaticTFwithRot(pose);
                    // success = arm.movself.e_p(pose);
                    pose1 = tool.calculateTargetPose(p.pose, arm.setPoint(std::vector<double>{0, 0, -0.02, 0, 0, 0}));
                    success = arm.move_p(pose1, success);
                    pose1 = tool.calculateTargetPose(p.pose, arm.setPoint(std::vector<double>{0, 0, 0.01, 0, 0, 0}));
                    success = arm.move_l(pose1, success);
                    arm.Set_Tool_DO(2, true);
                    pose1 = tool.calculateTargetPose(p.pose, arm.setPoint(std::vector<double>{0, 0, -0.05, 0, 0, 0}));
                    success = arm.move_l(pose1, success);
                    // success = arm.move_l(pose, success);
                    if (success)
                    {
                        success = arm.move_p(start_pose);
                        // 放置物品到车上

                        success = arm.move_l(arm.setPoint(std::vector<double>{-0.145, -0.137, 0.2, -3.1415, 0, 0}), success);
                        success = arm.move_l(arm.setPoint(std::vector<double>{-0.145, -0.137, 0.02, -3.1415, 0, 0}), success);
                        arm.Set_Tool_DO(2, false);
                        success = arm.move_l(arm.setPoint(std::vector<double>{-0.145, -0.137, 0.2, -3.1415, 0, 0}), success);
                    }

                    // success = arm.move_p(start_pose);
                    if (success)
                        break;
                    // }
                    ps.clear();
                }
            }
            if (success == false)
            {
                // arm.move_j(std::vector<double>{0, tool.degreesToRadians(50.00), tool.degreesToRadians(-50.00),
                //                                0, tool.degreesToRadians(-90), tool.degreesToRadians(180)});
                return success;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }

        return success;
    }
    bool openCabinet()
    {
        bool success;
        ROS_INFO_NAMED("openCabinet", "openCabinet");
        arm.setMaxVelocity(0.2);
        try
        {
            // 此段代码的相对位置为 x->z ,y->x,z->y
            ROS_INFO_NAMED("openCabinet", "移动到识别点");
            success = arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-86), tool.degreesToRadians(-62),
                                                     tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});
            geometry_msgs::Pose target, target1, target2, target3, target4;
            robot_msgs::Hand_Catch run;
            run.request.run = true;
            run.request.name = "drawerhandle";
            // object_client.call(run);
            if (object_client.call(run) && !run.response.positions.empty())
            {
                target = run.response.positions[1].pose;
                target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, tool.degreesToRadians(90), tool.degreesToRadians(90), 0});
                arm.Set_Tool_DO(2, false);

                ROS_INFO_NAMED("openCabinet", "1");
                target1 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, -0.1, 0, 0, tool.degreesToRadians(-90)}));
                // target1 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, tool.degreesToRadians(90)}));
                success = arm.move_p(target1, success);

                ROS_INFO_NAMED("openCabinet", "2");
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.11, 0, 0, 0}));
                success = arm.move_l(target1, success);
                arm.Set_Tool_DO(2, true);

                ROS_INFO_NAMED("openCabinet", "3");
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.23, 0, 0, 0}));
                success = arm.move_l(target1, success);
                arm.Set_Tool_DO(2, false);

                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.05, 0, 0, 0}));
                //  tool.publishStaticTFwithRot(target1);
                success = arm.move_l(target1, success);
                // 移动到柜子上方
                target3 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{-0.17, 0, 0, 0, 0, tool.degreesToRadians(90)}));
                //  tool.publishStaticTFwithRot(target3);
                success = arm.move_l(target3, success);

                // 开第二层抽屉
                // ROS_INFO_NAMED("openCabinet", "4");
                // target = run.response.positions[0].pose;s
                // target = arm.setPoint(std::vector<double>{target.position.x, target.position.y, target.position.z, tool.degreesToRadians(90), tool.degreesToRadians(90), 0});
                // target2 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, tool.degreesToRadians(90)}));
                // success = arm.move_l(target2, success);
                // target2 = tool.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, -0.11, 0, 0, 0}));
                // success = arm.move_l(target2, success);
                // arm.Set_Tool_DO(2, true);
                // target2 = tool.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, 0.23, 0, 0, 0}));
                // success = arm.move_l(target2, success);
                // arm.Set_Tool_DO(2, false);
                // target2 = tool.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0, 0, 0.05, 0, 0, 0}));
                // success = arm.move_l(target2, success);
                // target3 = tool.calculateTargetPose(target2, arm.setPoint(std::vector<double>{0.15, 0, 0, 0, 0, tool.degreesToRadians(-90)}));
                // success = arm.move_l(target3, success);
                // target3 = arm.setPoint(std::vector<double>{target.position.x - 0.10, target.position.y - 0.02, target.position.z - 0.04, 0, 0, 0});
                //  tool.publishStaticTF(target1);
                // ROS_INFO("Publish TF");
                // ros::Timer timer = nh.createTimer(ros::Duration(0.5), boost::bind(timerCallback, _1, target3));
                // 移动到观察点1（查看柜子桌面）
                // target4 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0.25, 0.13, -0.05, 0, 0, 0}));
                // //  tool.publishStaticTFwithRot(target4);
                // target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(-30), 0}));
                // //  tool.publishStaticTFwithRot(target4);
                // // success = arm.move_p(target4, success);
                // target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, tool.degreesToRadians(10), 0, 0}));
                // //  tool.publishStaticTFwithRot(target4);
                // success = arm.move_p(target4, success);
                // // 移动到观察点2 查看柜子内部
                target4 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(90), tool.degreesToRadians(-90)}));
                //  tool.publishStaticTFwithRot(target3);
                // success = arm.move_p(target3, success);
                target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, 0, 0, tool.degreesToRadians(90)}));
                //  tool.publishStaticTFwithRot(target3);
                success = arm.move_p(target4, success);
                ROS_INFO_NAMED("openCabinet", "抓取");
                target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{-0.08, 0, -0.03, 0, 0, 0}));
                // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0.18, 0, -0.05, 0, 0, 0}));
                //  tool.publishStaticTFwithRot(target3);
                success = arm.move_l(target4, success);
                // arm.Set_Tool_DO(2, false);

                // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{-0.2, 0, 0.05, 0, 0, 0}));
                //  tool.publishStaticTFwithRot(target3);
                // success = arm.move_l(target3, success);
                if (success)
                    success = colorCapture();
                // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(90), 0}));
                //  tool.publishStaticTFwithRot(target3);
                ROS_INFO_NAMED("openCabinet", "结束抓取");
                target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0.08, 0, 0.05, 0, 0, 0}));
                success = arm.move_l(target4, success);
                target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(-90), 0}));
                //  tool.publishStaticTFwithRot(target4);
                // 移动到合上抽屉的位置
                success = arm.move_p(target4,success);
                success = arm.move_l(target3, success);
                success = arm.move_l(target1, success);
                //  tool.publishStaticTFwithRot(target1);
                arm.Set_Tool_DO(2, true);
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0.02, 0, -0.295, 0, 0, 0}));
                success = arm.move_l(target1, success);

                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.295, 0, 0, 0}));
                success = arm.move_l(target1, success);
                target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0, 0, 0, tool.degreesToRadians(-90)}));
                success = arm.move_l(target1, success);
                if (!success)
                {
                    arm.go_home();
                }
            }
            if (success == false)
            {
                arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-86), tool.degreesToRadians(-62),
                                               tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});
                arm.go_home();
                return success;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }

        return success;
    }
    bool searchDestination()
    {
        double angle;
        double goal_x, goal_y;
        std::vector<double> joint_group_positions;
        arm.setMaxVelocity(0.2);
        int num = 0;
        auto call = [this](double s, double theta, double x, double y)
        {
            ROS_INFO_STREAM("active car");
            yd_msgs::Pose_Task ptr;
            ptr.request.Speed = s;
            ptr.request.PoseSend.theta = theta;
            ptr.request.PoseSend.x = x;
            ptr.request.PoseSend.y = y;
            ptr.request.Id = 0;
            arm.go_home();
            carControl.call(ptr);
        };
        // arm.move_j(std::vector<double>{tool.degreesToRadians(0), tool.degreesToRadians(28), tool.degreesToRadians(-78),
        //                                tool.degreesToRadians(0), tool.degreesToRadians(-71), tool.degreesToRadians(180)});
        arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-86), tool.degreesToRadians(-62),
                                       tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});

        state = true;

        arm.getCurrentJoint(joint_group_positions);
        angle = joint_group_positions[0];

        // 当角度x,y其中一个不能给0

        while (num < 10)
        {
            num++;
            robot_msgs::Objection_Detect goal;
            goal.request.run = true;
            object_client_realtime.call(goal);
            if (goal.response.success)
            {
                break;
            }
            if (goal.response.result)
            {
                geometry_msgs::Pose pose = goal.response.position.pose;
                // 目的地距离目标物品为（0.38，0.386），考虑车的误差，比实际要大，通过相减获得偏移量
                goal_x = pose.position.x - 0.435;
                goal_y = pose.position.y + 0.40;
                pose.position.x = goal_x;
                pose.position.y = goal_y;
                tool.publishStaticTFwithRot(pose);
                call(0.2, 0, goal_x, goal_y);
                ros::Duration(20).sleep();
                arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-86), tool.degreesToRadians(-62),
                                               tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});
                object_client_realtime.call(goal);
                if (goal.response.success)
                {
                    break;
                }
            }
            else
            {
                if (state)
                {
                    arm.move_j(std::vector<double>{tool.degreesToRadians(0), tool.degreesToRadians(28), tool.degreesToRadians(-78),
                                                   tool.degreesToRadians(0), tool.degreesToRadians(-71), tool.degreesToRadians(180)});
                    state = false;
                    arm.getCurrentJoint(joint_group_positions);
                    angle = joint_group_positions[0];
                }
                angle += tool.degreesToRadians(5);
                joint_group_positions[0] = angle;
                arm.move_j(joint_group_positions);
            }
        }
        if (num < 10)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void callTaskAction(const robot_msgs::Call_TaskGoalConstPtr &goal)
    {
        robot_msgs::Call_TaskResult rep;
        for (auto task : goal->taskId)
        {
            int Id = (int)task - 48;
            switch (Id)
            {
            case 1:
                ROS_INFO_NAMED("TASK", "Start taskId 1 named 'open door'. ");
                rep.succeed = openDoor();
                break;
            case 2:
                ROS_INFO_NAMED("TASK", "Start taskId 2 named 'open cabinet'. ");
                rep.succeed = openCabinet();
                break;
            case 3:
                ROS_INFO_NAMED("TASK", "Start taskId 3 named 'color capture'. ");
                rep.succeed = colorCapture();
                break;
            case 4:
                ROS_INFO_NAMED("TASK", "Start taskId 4 named 'search Destination'. ");
                rep.succeed = searchDestination();
                break;
            default:
                break;
            }
        }

        server->setSucceeded(rep);
    }

    bool callTask(robot_msgs::Task_Call::Request &req,
                  robot_msgs::Task_Call::Response &rep)
    {
        for (auto task : req.taskId)
        {
            int Id = (int)task - 48;
            switch (Id)
            {
            case 1:
                ROS_INFO_NAMED("TASK", "Start taskId 1 named 'open door'. ");
                rep.succeed = openDoor();
                break;
            case 2:
                ROS_INFO_NAMED("TASK", "Start taskId 2 named 'open cabinet'. ");
                rep.succeed = openCabinet();
                break;
            case 3:
                ROS_INFO_NAMED("TASK", "Start taskId 3 named 'color capture'. ");
                rep.succeed = colorCapture();
                break;
            case 4:
                ROS_INFO_NAMED("TASK", "Start taskId 4 named 'search Destination'. ");
                rep.succeed = searchDestination();
                break;
            default:
                break;
            }
        }

        rep.succeed = true;
        return true;
    }

private:
    ros::NodeHandle nh_;
    MoveitServer arm;
    ros::Timer timer;
    // ros::Subscriber a;
    Server *server;
    // Client *object_client_realtime;
    ros::Publisher iswork, current_state;
    ros::ServiceClient object_client, color_client, carControl,
        resetmap, toggle, get_scene_cloud_client, get_map_cloud_client, predict_grasps_client, object_client_realtime;
    ros::ServiceServer CallTask;
    std_msgs::String statemsg;
    robotTool tool;
    bool state;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "ArmControlCenter");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    std::string PLANNING_GROUP = "arm";
    ControlCenter con(nh, PLANNING_GROUP);
    ros::waitForShutdown();
    return 0;
}
// -4.825333019521214,-1.5727815063613566,-0.9999972558986754,0.0023426897189369736};

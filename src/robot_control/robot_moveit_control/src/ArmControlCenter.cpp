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
#include <yd_msgs/MoveGlobalTargetAction.h>
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
        carControl = nh.serviceClient<yd_msgs::Pose_Task>("/Rm_TargetPose");
        carCon = new actionlib::SimpleActionClient<yd_msgs::MoveGlobalTargetAction>(nh, "ydrobot_controls", true);
        // carCon->waitForServer();
        object_client_realtime = nh.serviceClient<robot_msgs::Objection_Detect>("object_realtime_detect");

        ROS_INFO_NAMED("TASK", "initialize_service_client. ");
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
                    // 目标表面位置
                    // target = arm.setPoint(std::vector<double>{p.pose.position.x, p.pose.position.y, p.pose.position.z, 0, 0, 0});
                    // // 夹取目标位置
                    // target3 = arm.setPoint(std::vector<double>{target.position.x , target.position.y , target.position.z - 0.03, 0, 0, 0});
                    // tool.publishStaticTF(target3);
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
                    success = arm.move_p(pose1, success); // 目标上方
                    pose1 = tool.calculateTargetPose(p.pose, arm.setPoint(std::vector<double>{0, 0, 0.01, 0, 0, 0}));
                    success = arm.move_l(pose1, success); // 夹取位置
                    arm.Set_Tool_DO(2, true);
                    pose1 = tool.calculateTargetPose(p.pose, arm.setPoint(std::vector<double>{0, 0, -0.05, 0, 0, 0}));
                    success = arm.move_l(pose1, success); // 抬起物体
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
            success = arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-77), tool.degreesToRadians(-68),
                                                     tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});

            // 检测判断能否抓到柜子
            robot_msgs::Objection_Detect goal;
            goal.request.run = true;
            object_client_realtime.call(goal);

            // 如果检测成功，继续执行后续动作
            if (goal.response.success)
            {
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
                    // 移动到抽屉把手预抓取位置
                    ROS_INFO_NAMED("openCabinet", "1");
                    target1 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, -0.1, 0, 0, tool.degreesToRadians(-90)}));
                    // target1 = tool.calculateTargetPose(target, arm.setPoint(std::vector<double>{0, 0, 0.1, 0, 0, tool.degreesToRadians(90)}));
                    success = arm.move_p(target1, success);

                    // 移动到把手抓取位置
                    ROS_INFO_NAMED("openCabinet", "2");
                    target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.11, 0, 0, 0}));
                    success = arm.move_l(target1, success);
                    arm.Set_Tool_DO(2, true); // 关闭夹爪

                    // 向外拉抽屉
                    ROS_INFO_NAMED("openCabinet", "3");
                    target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.22, 0, 0, 0}));
                    success = arm.move_l(target1, success);
                    arm.Set_Tool_DO(2, false);
                    // 夹爪退出把手位置
                    // target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.05, 0, 0, 0}));
                    // tool.publishStaticTFwithRot(target1);
                    // success = arm.move_l(target1, success);
                    std::vector<double> joint_group_positions;
                    arm.getCurrentJoint(joint_group_positions);
                    joint_group_positions[0] += tool.degreesToRadians(10); // 调整角度使其退出位置
                    arm.move_j(joint_group_positions);
                    // 移动到过渡姿态

                    // success = arm.move_j(std::vector<double>{tool.degreesToRadians(46), tool.degreesToRadians(-77), tool.degreesToRadians(-68),
                    //                                  tool.degreesToRadians(132), tool.degreesToRadians(112), tool.degreesToRadians(155)});

                    target3 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, -0.15, -0.05, 0, 0, tool.degreesToRadians(90)}));
                    // tool.publishStaticTFwithRot(target3,"pose1");
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

                    // 移动到过渡姿态
                    success = arm.move_j(std::vector<double>{tool.degreesToRadians(11), tool.degreesToRadians(-17), tool.degreesToRadians(-70),
                                                             tool.degreesToRadians(0), tool.degreesToRadians(-94), tool.degreesToRadians(259)},
                                         success);

                    // 移动到抽屉上方
                    target4 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, -0.05, 0, 0, tool.degreesToRadians(90), 0}));
                    success = arm.move_p(target4, success);
                    ROS_INFO_NAMED("openCabinet", "抓取");
                    // 移动到夹取观察点2，查看抽屉内部
                    target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{-0.08, 0, 0, 0, 0, 0}));
                    // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0.18, 0, -0.05, 0, 0, 0}));
                    // tool.publishStaticTFwithRot(target4,"pose3");
                    success = arm.move_l(target4, success);
                    // arm.Set_Tool_DO(2, false);

                    // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{-0.2, 0, 0.05, 0, 0, 0}));
                    //  tool.publishStaticTFwithRot(target3);
                    // success = arm.move_l(target3, success);
                    // 抓取抽屉里的目标物体
                    if (success)
                        success = colorCapture();
                    // target3 = tool.calculateTargetPose(target3, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(90), 0}));
                    //  tool.publishStaticTFwithRot(target3);
                    ROS_INFO_NAMED("openCabinet", "结束抓取");

                    // 移动回抽屉上方
                    target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0.08, 0, 0, 0, 0, 0}));
                    success = arm.move_l(target4, success);

                    // 移动到合上抽屉的位置
                    // target3 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, 0, 0, 0}));
                    // tool.publishStaticTFwithRot(target4,"pose4");
                    // success = arm.move_p(target3, success); // 过渡姿态
                    target4 = tool.calculateTargetPose(target4, arm.setPoint(std::vector<double>{0, 0, 0, 0, tool.degreesToRadians(-90), 0}));
                    // tool.publishStaticTFwithRot(target4,"pose4");
                    success = arm.move_p(target4, success); // 水平姿态
                    // success = arm.move_j(std::vector<double>{tool.degreesToRadians(23), tool.degreesToRadians(-81), tool.degreesToRadians(-94),
                    //                                 tool.degreesToRadians(-77), tool.degreesToRadians(-88), tool.degreesToRadians(263)}); // 过渡姿态
                    target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.03, 0, 0, 0}));
                    success = arm.move_p(target1, success); // 预推姿态
                    // tool.publishStaticTFwithRot(target1);
                    arm.Set_Tool_DO(2, true);
                    // 关闭抽屉
                    target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0.245, 0, 0, 0}));
                    success = arm.move_l(target1, success);

                    // 完成动作，回到原位
                    target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, -0.20, 0, 0, 0}));
                    success = arm.move_l(target1, success);
                    // target1 = tool.calculateTargetPose(target1, arm.setPoint(std::vector<double>{0, 0, 0, 0, 0, tool.degreesToRadians(90)}));
                    // success = arm.move_l(target1, success);
                    if (success)
                    {
                        success = arm.go_home();
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
        }
        catch (const std::exception &e)
        {
            ROS_ERROR_NAMED("openCabinet", "Exception: %s", e.what());
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
            yd_msgs::MoveGlobalTargetGoal ptr;
            ptr.Speed = s;
            ptr.PoseSend.theta = theta;
            ptr.PoseSend.x = x;
            ptr.PoseSend.y = y;
            ptr.Id = 0;
            arm.go_home();
            carCon->sendGoalAndWait(ptr, ros::Duration(60));
            // carControl.call(ptr);

            // carCon->waitForResult(ros::Duration(60));
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
                // ros::Duration(20).sleep();
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
    actionlib::SimpleActionClient<yd_msgs::MoveGlobalTargetAction> *carCon;
    ros::Publisher iswork, current_state;
    ros::ServiceClient object_client, color_client, carControl, object_client_realtime;
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

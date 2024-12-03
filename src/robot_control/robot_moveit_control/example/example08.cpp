/*
 * 实现机械臂小车对可回收垃圾【瓶子管子等物品的回收抓取和丢弃】
 */

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <robot_msgs/Hand_Catch.h>
#include <robot_msgs/Task_Call.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <MoveitServer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yd_msgs/MoveLocalTargetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/Call_TaskAction.h>
#include <robot_msgs/ObjectionRealTimeDetect.h>
#include <robotTool.h>

typedef actionlib::SimpleActionClient<yd_msgs::MoveLocalTargetAction> CarClient;

class ArmControlCenter
{
public:
    ArmControlCenter() : carCon_("ydrobot_controls", true)
    {
        // 订阅检测数据
        sub_ = nh_.subscribe("realtime_detect", 10, &ArmControlCenter::detectCallback, this);

        object_client = nh_.serviceClient<robot_msgs::Hand_Catch>("object_detect");

        // 初始化小车控制
        ROS_INFO("Waiting for car control server...");
        carCon_.waitForServer();
        hasProcessedData_ = false;
        detectionThread_ = std::thread(&ArmControlCenter::detect, this);
        std::string PLANNING_GROUP = "arm";  
        // 初始化机械臂控制
        moveitServer_ = std::make_shared<MoveitServer>(PLANNING_GROUP);
        ROS_INFO("ArmControlCenter initialized successfully.");
    }
    void detect()
    {
        robot_msgs::Hand_Catch run;
        std::vector<geometry_msgs::PoseStamped> positions;
        std::vector<std::string> labels;
        ros::Rate rate(1); // 控制检测频率为2Hz
        while (!stopThread_)
        {
            run.request.name = "";
            if (object_client.call(run))
            {
                positions = run.response.positions;
                labels = run.response.labels;

                if (!positions.empty() && !labels.empty())
                {
                    const auto &label = labels[0];
                    const auto &pose = positions[0];

                    if (label == "can" || label == "plastic bottle")
                    {
                        if (std::abs(pose.pose.position.x) < 1.6 && std::abs(pose.pose.position.y) < 1.6)
                        {
                            if (!hasProcessedData_) // 确保当前数据尚未被处理
                            {
                                ROS_INFO("Label: %s, Position: x=%.2f, y=%.2f, z=%.2f",
                                         label.c_str(),
                                         pose.pose.position.x,
                                         pose.pose.position.y,
                                         pose.pose.position.z);

                                hasProcessedData_ = true; // 标记数据已处理
                                moveCarToTarget(pose.pose.position.x - 0.18, pose.pose.position.y);
                            }
                        }
                    }
                }
            }

            rate.sleep(); // 控制检测频率，避免高 CPU 占用
        }
    }

    void detectCallback(const robot_msgs::ObjectionRealTimeDetect::ConstPtr &msg)
    {
        if (hasProcessedData_) // 已处理数据，直接返回
            return;

        // 解析检测结果
        std::vector<std::string> labels = msg->labels;
        std::vector<geometry_msgs::PoseStamped> positions = msg->positions;

        if (labels.empty() || positions.empty())
            return;

        ROS_INFO("Received detection data:");
        for (size_t i = 0; i < 1; ++i)
        {
            const auto &label = labels[i];
            const auto &pose = positions[i];

            // 仅处理指定类别的物体
            if (label == "can" || label == "plastic bottle")
            {
                if (std::abs(pose.pose.position.x) < 1.6 && std::abs(pose.pose.position.y) < 1.6)
                {
                    ROS_INFO("Label: %s, Position: x=%.2f, y=%.2f, z=%.2f",
                             label.c_str(),
                             pose.pose.position.x,
                             pose.pose.position.y,
                             pose.pose.position.z);
                    hasProcessedData_ = true; // 标记已处理
                    moveCarToTarget(pose.pose.position.x - 0.18, pose.pose.position.y);

                }
            }
        }
    }
    ~ArmControlCenter()
    {
        // 停止线程
        stopThread_ = true;
        if (detectionThread_.joinable())
        {
            detectionThread_.join();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    CarClient carCon_;
    std::atomic<bool> stopThread_;       // 用于控制线程退出
    std::atomic<bool> hasProcessedData_; // 标记是否已处理数据
    ros::ServiceClient object_client, color_client;
    std::thread detectionThread_;
    std::shared_ptr<MoveitServer> moveitServer_;

    void moveCarToTarget(double x, double y)
    {
        yd_msgs::MoveLocalTargetGoal goal;
        goal.Speed = 0.2;
        goal.PoseSend.x = x;
        goal.PoseSend.y = y;
        goal.PoseSend.theta = atan(y/x);
        goal.Id = 0;

        ROS_INFO("Moving car to target: x=%.2f, y=%.2f", x, y);
        carCon_.sendGoalAndWait(goal, ros::Duration(60));
        hasProcessedData_ = false;
    }

    // bool grabObject(const geometry_msgs::PoseStamped &pose)
    // {
    //     ROS_INFO("Planning to grab object at x=%.2f, y=%.2f, z=%.2f",
    //              pose.pose.position.x,
    //              pose.pose.position.y,
    //              pose.pose.position.z);

    //     // 调用 MoveItServer 执行抓取操作
    //     if (moveitServer_->moveToPose(pose.pose))
    //     {
    //         ROS_INFO("Object grabbed successfully.");
    //         return true;
    //     }
    //     ROS_WARN("Failed to grab object.");
    //     return false;
    // }

    // void discardObject()
    // {
    //     geometry_msgs::Pose discardPose;
    //     discardPose.position.x = 0.5; // 回收点位置
    //     discardPose.position.y = 0.0;
    //     discardPose.position.z = 0.5;

    //     ROS_INFO("Discarding object...");
    //     moveitServer_->moveToPose(discardPose);
    //     ROS_INFO("Object discarded successfully.");
    // }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ArmControlCenter");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);

    ArmControlCenter armControlCenter;
    spinner.start();
    // ros::spin();
    ros::waitForShutdown();
    return 0;
}

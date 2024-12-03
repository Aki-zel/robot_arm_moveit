#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_msgs/TaskExecutionAction.h>
#include <yaml-cpp/yaml.h> // 用于处理 YAML 格式
#include <MoveitServer.h>
#include <robotTool.h>
#include <yd_msgs/MoveLocalTargetAction.h>
#include <robot_msgs/ObjectionRealTimeDetect.h>
#include <robot_msgs/Hand_Catch.h>
#include <thread>
#include <atomic>

class TaskExecutionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<robot_msgs::TaskExecutionAction> as_; // 创建一个 ActionServer
    std::string action_name_;
    robot_msgs::TaskExecutionFeedback feedback_; // 状态反馈
    robot_msgs::TaskExecutionResult result_;     // 执行结果
    ros::ServiceClient color_detect, object_detect;
    ros::Subscriber realtime_detect;
    std::unique_ptr<MoveitServer> arm;
    std::unique_ptr<robotTool> tool;
    std::thread task_thread_;
    std::atomic<bool> is_running_, success; // 控制任务是否在运行

public:
    TaskExecutionServer(std::string name) : as_(nh_, name, boost::bind(&TaskExecutionServer::executeCB, this, _1), false), // 设置回调函数
                                            action_name_(name)
    {
        ROS_INFO("Start TaskExecutionServer ...");
        std::string PLANNING_GROUP = "arm";
        is_running_ = false;
        success = false;
        color_detect = nh_.serviceClient<robot_msgs::Hand_Catch>("color_detect");
        object_detect = nh_.serviceClient<robot_msgs::Hand_Catch>("object_detect");
        arm = std::make_unique<MoveitServer>(PLANNING_GROUP);
        tool = std::make_unique<robotTool>();
        arm->setMaxVelocity(0.6, 0.4);
        as_.start(); // 启动 Action Server
        ROS_INFO("Start TaskExecutionServer Ok");
        ROS_INFO("The TaskExecutionServer is running on action: %s", name.c_str());
    }
    ~TaskExecutionServer()
    {
        if (task_thread_.joinable())
        {
            task_thread_.join();
        }
    }

    void executeCB(const robot_msgs::TaskExecutionGoalConstPtr &goal)
    {
        ROS_INFO("%s: Executing task: %s with parameters: %s", action_name_.c_str(), goal->task_type.c_str(), goal->parameters.c_str());
        is_running_ = true;

        // 根据任务类型解析不同的 YAML 格式
        if (goal->task_type == "charge")
        {
            ROS_INFO("Charge ... ");
            // 执行机器人移动任务
            task_thread_ = std::thread(&TaskExecutionServer::performChargeTask, this);
        }
        else if (goal->task_type == "discharge")
        {
            ROS_INFO("DisCharge ... ");
            task_thread_ = std::thread(&TaskExecutionServer::performDisChargeTask, this);
        }
        else if (goal->task_type == "robot_rotate")
        {
            // 解析机器人旋转任务的 YAML
            YAML::Node task_params = YAML::Load(goal->parameters);
            double angle = task_params["angle"].as<double>();
            double speed = task_params["speed"].as<double>();

            ROS_INFO("Rotate Angle: %.2f, Speed: %.2f", angle, speed);

            // 执行机器人旋转任务
            task_thread_ = std::thread(&TaskExecutionServer::performRotateTask, this, angle, speed);
        }
        else if (goal->task_type == "robot_pick")
        {
            // 解析机器人抓取任务的 YAML
            YAML::Node task_params = YAML::Load(goal->parameters);
            std::string object_id = task_params["object_id"].as<std::string>();
            std::string location = task_params["location"].as<std::string>();

            ROS_INFO("Pick Object: %s, Location: %s", object_id.c_str(), location.c_str());

            // 执行机器人抓取任务
            task_thread_ = std::thread(&TaskExecutionServer::performPickTask, this, object_id, location);
        }
        else if (goal->task_type == "stop")
        {
            ROS_INFO("Stop the current task... ");
            arm->go_home();
            is_running_ = false;
        }
        else
        {
            ROS_ERROR("Unknown task type: %s", goal->task_type.c_str());
            as_.setAborted();
            return;
        }

        // 定期检查抢占请求
        while (is_running_)
        {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted(); // 设置为已取消
                is_running_ = false;
                if (task_thread_.joinable())
                {
                    task_thread_.join();
                }
                return;
            }
            // 定期发送反馈
            feedback_.status = "Task is running... ";
            as_.publishFeedback(feedback_);
            ros::Duration(1.0).sleep(); // 模拟任务执行的延时
        }

        // 返回执行结果
        if (success)
        {
            result_.success = true;
            result_.result = "Task completed successfully.";
            ROS_INFO("%s: Task completed.", action_name_.c_str());
            if (task_thread_.joinable())
            {
                task_thread_.join();
            }
            as_.setSucceeded(result_);
        }
        else
        {
            result_.success = false;
            result_.result = "Task failed.";
            if (task_thread_.joinable())
            {
                task_thread_.join();
            }
            as_.setSucceeded(result_);
        }
    }

    void performChargeTask()
    {
        tool->setGripperForce(100);
        success = arm->move_j(-128, 4, 89, 0, 84, 63);
        success = arm->move_j(0, 3, 66, 0, 110, 0, success && is_running_);
        robot_msgs::Hand_Catch g;
        g.request.name = "qrcode";
        g.request.isgrasp = false;
        if (object_detect.call(g) && g.response.positions.size() > 0)
        {
            auto pose = g.response.positions[0].pose;
            auto labels = g.response.labels;
            pose = tool->calculateTargetTransform(pose, tool->lookupTransform("Link6", "gripper").transform);
            tool->setGripperPosition(40);
            auto prepose = tool->moveFromPose(pose, -0.1);
            tool->publishStaticTFwithRot(prepose, "prepose");
            success = arm->move_l(prepose, success && is_running_);
            success = arm->move_l(pose, success && is_running_);
            tool->setGripperPosition(0);
            ros::Duration(0.5).sleep();
            success = arm->move_l(prepose, success && is_running_);
        }
        success = arm->move_j(-128, 4, 89, 0, 84, 63, success && is_running_);
        auto current_pose = arm->getCurrent_Pose();
        // tool->setGripperPosition(30);
        ROS_INFO("获取当前的坐标点");
        ROS_INFO("位置信息: x: %f, y: %f, z: %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        ROS_INFO("方向信息: qx: %f, qy: %f, qz: %f, qw: %f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
        auto p1 = current_pose;
        p1.position.z -= 0.095;
        success = arm->move_l(p1, success && is_running_);
        tool->setGripperPosition(30);
        success = arm->move_l(current_pose, success && is_running_);
        is_running_ = false;
    }
    void performDisChargeTask()
    {
        tool->setGripperForce(100);
        success = arm->move_j(0, 3, 66, 0, 110, 0);
        robot_msgs::Hand_Catch g;
        g.request.name = "qrcode";
        g.request.isgrasp = false;
        if (object_detect.call(g) && g.response.positions.size() > 0)
        {
            auto pose = g.response.positions[0].pose;
            auto labels = g.response.labels;
            pose = tool->moveFromPose(pose, -0.055);
            pose = tool->calculateTargetTransform(pose, tool->lookupTransform("Link6", "gripper").transform);
            success = arm->move_j(-128, 4, 89, 0, 84, 63);
            tool->setGripperPosition(30);
            auto current_pose = arm->getCurrent_Pose();
            // tool->setGripperPosition(30);
            ROS_INFO("获取当前的坐标点");
            ROS_INFO("位置信息: x: %f, y: %f, z: %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
            ROS_INFO("方向信息: qx: %f, qy: %f, qz: %f, qw: %f", current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w);
            auto p1 = current_pose;
            p1.position.z -= 0.092; // 直接减是向下移动
            success = arm->move_l(p1, success && is_running_);
            tool->setGripperPosition(0);
            ros::Duration(0.5).sleep();
            success = arm->move_l(current_pose, success && is_running_);
            success = arm->move_j(0, 3, 66, 0, 110, 0, success && is_running_);
            auto prepose = tool->moveFromPose(pose, -0.1);
            tool->publishStaticTFwithRot(prepose, "prepose");
            success = arm->move_l(prepose, success && is_running_);
            success = arm->move_l(pose, success && is_running_);
            tool->setGripperPosition(40);
            ros::Duration(0.5).sleep();
            success = arm->move_l(prepose, success && is_running_);
            success = arm->move_j(-128, 4, 89, 0, 84, 63, success && is_running_);
        }
        is_running_ = false;
    }

    // 执行机器人旋转任务
    void performRotateTask(double angle, double speed)
    {
        while (ros::ok() && is_running_)
        {
            ROS_INFO("Rotating with angle: %.2f, speed: %.2f", angle, speed);
            ros::Duration(1.0).sleep(); // 模拟旋转过程
        }
        success = true;
        is_running_ = false;
    }

    // 执行机器人抓取任务
    void performPickTask(const std::string &object_id, const std::string &location)
    {
        while (ros::ok() && is_running_)
        {
            ROS_INFO("Picking object: %s at location: %s", object_id.c_str(), location.c_str());
            ros::Duration(1.0).sleep(); // 模拟抓取过程
        }
        success = true;
        is_running_ = false;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task_execution_server");

    ros::AsyncSpinner spinner(2);
    spinner.start();
    TaskExecutionServer server("task_execution");
    ros::waitForShutdown();
    return 0;
}

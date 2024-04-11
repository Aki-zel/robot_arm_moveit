#ifndef ArmController_H
#define ArmController_H
#include <ros/ros.h>
#include <iostream>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <cubicSpline.h>
#include <vector>
#include <algorithm>
#include <rm_base.h>
#include <rm_service.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <thread>
// #include <mwrobot_msgs/armgradm.h>

struct JointData
{
    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> accelerations;
};
struct RetData
{
    JointData joint[6];
    std::vector<double> time;
    int node;
    int link;
};
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;

class ArmController
{
private:
    RetData ret;                // 用于设置收发的值
    Server server;              // 服务端
    bool point_changed = false; // 判断路点数据是否改变
    ros::Publisher target_pub;  // Real robot
    ros::Publisher pub_getArmStateTimerSwitch;
    int current = 0;
    ros::Timer State_Timer;
    RM_Service service;
    int sockets;
    ros::Publisher statePub;
    std::string ip;
    ros::Subscriber handserver;

public:
    ArmController(std::string serverName, ros::NodeHandle nh);
    ~ArmController();
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goalPtr); // 动作目标回调函数
    void timer_callback();
    void sendGoaltra();
    void sentCurrentStateThread();
    bool linkToarm();
    void handcallback(const std_msgs::Bool::ConstPtr &msg);

};

#endif
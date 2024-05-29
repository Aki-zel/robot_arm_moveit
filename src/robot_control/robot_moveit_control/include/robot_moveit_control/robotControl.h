#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <rm_msgs/Tool_Digital_Output.h>
#include <MoveitServer.h>
#include <robot_msgs/MoveL_Data.h>
#include <std_msgs/Float32MultiArray.h>
class robotControl
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber moveJ_sub;
    ros::Subscriber moveL_sub;
    ros::Subscriber moveP_sub;

    ros::Publisher moveJ_pub;
    ros::Publisher moveL_pub;
    ros::Publisher moveP_pub;
    MoveitServer *moveit_server;

public:
    robotControl(/* args */);
    void MoveJ_cmd(const float pose[]);
    void MoveJ_cmd(const std::vector<double> pose);
    void MoveP_cmd(const double x, const double y, const double z);
    void MoveP_cmd(const geometry_msgs::Pose target);
    void MoveL_cmd(robot_msgs::MoveL_Data msgs);
    void Close();
    ~robotControl();

private:
    void MoveJ_cmd_callback(const std_msgs::Float32MultiArrayConstPtr joint);
    void MoveP_cmd_callback(const geometry_msgs::PoseConstPtr pose);
    void MoveL_cmd_callback(const robot_msgs::MoveL_DataConstPtr msgs);
};


#endif
#ifndef _CYCLOID_H
#define _CYCLOID_H // 定义 _CYCLOID_H 宏，防止重复包含

#include <ros/ros.h> // 包含 ROS 核心头文件
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <queue>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <string>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/RobotState.h>
#include <sensor_msgs/JointState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <arm_serial.h> // 包含自定义头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int16.h>

class Cycloid
{
private:
    std::string PlnningGroup;
    moveit::planning_interface::MoveGroupInterface interface;
    moveit::planning_interface::MoveGroupInterface::Plan myplan;
    const moveit::core::JointModelGroup *joint_model_group;
    ros::NodeHandle nh;
    ros::Publisher pub; // 末端爪子的控制话题

public:
    Cycloid(std::string GroupName);
    void Plan(moveit::planning_interface::MoveGroupInterface::Plan plan);
    void Execute();
    void Move(double x, double y, double z);
    void Move(std::vector<geometry_msgs::Pose> Points);
    void Move(geometry_msgs::Pose Point);
    void Move(std::string pose);
    void Automatic(double x, double y, double z, double point[]);
    geometry_msgs::Pose setPoint(double x, double y, double z);
    geometry_msgs::Pose setPoint(double x, double y, double z, double dx, double dy, double dz);
    geometry_msgs::Pose setPoint(double x, double y, double z, double dx, double dy, double dz,double dw);
    geometry_msgs::Pose getCurrentEndPoint();
    ~Cycloid();
};

#endif
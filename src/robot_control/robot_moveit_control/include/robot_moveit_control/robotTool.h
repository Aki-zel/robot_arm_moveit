#ifndef _ROBOT_TRANSFORM_H
#define _ROBOT_TRANSFORM_H
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <std_msgs/Int16.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <vgn/GraspConfig.h>
class robotTool
{
private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::StaticTransformBroadcaster stbroadcaster;

public:
    robotTool();
    ~robotTool();
    void publishStaticTF(const geometry_msgs::Pose &p, std::string name = "task");
    geometry_msgs::Pose transPose(geometry_msgs::Pose p, std::string source_frame = "task", std::string target_frame = "base_link_rm");
    void publishStaticTFwithRot(const geometry_msgs::Pose &p, std::string name = "pose");
    void publishGraspPoses(const std::vector<vgn::GraspConfig> &grasps);
    geometry_msgs::Pose calculateTargetPose(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &trans_pose);
    geometry_msgs::Pose calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform);
    geometry_msgs::Pose moveFromPose(const geometry_msgs::Pose &pose, double distance);
    double round(double num, int exponent);
    double degreesToRadians(double degrees);
   
};

robotTool::robotTool()
{
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

robotTool::~robotTool()
{
    
}

#endif
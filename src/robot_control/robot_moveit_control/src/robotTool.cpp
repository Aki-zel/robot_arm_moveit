#include <robotTool.h>
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
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
/// @brief 发布相对于机械臂基座标的TF坐标不包含位姿
/// @param p 类型geometry_msgs::Pose 含义：需要被发布的坐标点
/// @param name 类型std::string 含义：需要发布的坐标点名称
void robotTool::publishStaticTF(const geometry_msgs::Pose &p, std::string name)
{
    ROS_INFO("Publish TF task");

    // 创建坐标系信息
    geometry_msgs::TransformStamped ts;
    // 设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "xMate3_base";
    // 设置子级坐标系
    ts.child_frame_id = name;
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
    stbroadcaster.sendTransform(ts);
}
/// @brief
/// @param p geometry_msgs::Pose
/// @param source_frame std::string 原坐标系，数据来源的frame
/// @param target_frame std::string 数据应转换到的frame
/// @return geometry_msgs::Pose
geometry_msgs::Pose robotTool::transPose(geometry_msgs::Pose p, std::string source_frame, std::string target_frame)
{
    geometry_msgs::PoseStamped pose_stamped_in;
    pose_stamped_in.pose = p;
    pose_stamped_in.header.frame_id = source_frame;
    pose_stamped_in.header.stamp = ros::Time(0); // 使用最新的变换

    geometry_msgs::PoseStamped pose_stamped_out;
    try
    {
        // 1参数 是target_frame，这里理解为 数据应转换到的frame, 也就是 tf 的 frame_id ； 2 是source_frame ，这里理解为 数据来源的frame,
        // 等待变换可用
        tfBuffer.canTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
        // 获取变换
        tfBuffer.transform(pose_stamped_in, pose_stamped_out, target_frame);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform warning: %s", ex.what());
    }
    return pose_stamped_out.pose;
}

/// @brief 发布相对于机械臂基座标的TF坐标并且包含位姿
/// @param p 类型geometry_msgs::Pose 含义：需要被发布的坐标点
/// @param name 类型std::string 含义：需要发布的坐标点名称
void robotTool::publishStaticTFwithRot(const geometry_msgs::Pose &p, std::string name)
{
    ROS_INFO("Publish TF Pose");
    // 创建坐标系信息
    geometry_msgs::TransformStamped ts;
    // 设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";
    // 设置子级坐标系
    ts.child_frame_id = name;
    // 设置子级相对于父级的偏移量
    ts.transform.translation.x = p.position.x;
    ts.transform.translation.y = p.position.y;
    ts.transform.translation.z = p.position.z;
    ts.transform.rotation = p.orientation;
    // 广播器发布坐标系信息
    stbroadcaster.sendTransform(ts);
    ros::Duration(1).sleep();
}
/// @brief 用于计算两个坐标的转换，将trans_pose转换应用到target_pose
/// @param target_pose geometry_msgs::Pose
/// @param trans_pose geometry_msgs::Pose
/// @return geometry_msgs::Pose
geometry_msgs::Pose robotTool::calculateTargetPose(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &trans_pose)
{
    tf2::Transform target_tf;
    tf2::fromMsg(target_pose, target_tf);

    tf2::Transform relative_tf;
    tf2::fromMsg(trans_pose, relative_tf);

    tf2::Transform end_target_tf = target_tf * relative_tf;

    geometry_msgs::Pose end_target_pose;
    tf2::toMsg(end_target_tf, end_target_pose);
    return end_target_pose;
}

/// @brief 用于计算坐标和转换矩阵的转换，将relative_transform转换应用到target_pose
/// @param target_pose geometry_msgs::Pose
/// @param trans_pose geometry_msgs::Transform
/// @return geometry_msgs::Pose
geometry_msgs::Pose robotTool::calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform)
{
    tf2::Transform target_tf;
    tf2::fromMsg(target_pose, target_tf);

    tf2::Transform relative_tf;
    tf2::fromMsg(relative_transform, relative_tf);

    tf2::Transform end_target_tf = target_tf * relative_tf.inverse();

    geometry_msgs::Pose end_target_pose;
    tf2::toMsg(end_target_tf, end_target_pose);
    return end_target_pose;
}

/// @brief 用于简易的计算相对于当前坐标后退一定距离的坐标
/// @param pose geometry_msgs::Pose
/// @param distance double
/// @return geometry_msgs::Pose
geometry_msgs::Pose robotTool::moveFromPose(const geometry_msgs::Pose &pose, double distance)
{
    tf2::Transform pose_transform;
    tf2::fromMsg(pose, pose_transform);
    // 构造一个后退的变换矩阵
    tf2::Vector3 backward_vector(0.0, 0.0, distance); // 假设后退方向沿Z轴
    tf2::Vector3 transformed_vector = pose_transform.getBasis() * backward_vector;

    tf2::Transform back_transform;
    back_transform.setIdentity();
    back_transform.setOrigin(transformed_vector);
    tf2::Transform result_transform = back_transform * pose_transform;
    geometry_msgs::Pose result_pose;
    tf2::toMsg(result_transform, result_pose);
    // 将目标位置应用后退变换矩阵
    return result_pose;
}
double robotTool::round(double num, int exponent) // 四舍五入浮点数
{
    double multiplied = std::round(num * std::pow(10, exponent));
    double result = multiplied / std::pow(10, exponent);
    return result;
}
double robotTool::degreesToRadians(double degrees)
{
    return round((degrees * M_PI / 180.0), 10);
}
void robotTool::setGripperPosition(const int pose){
    ROS_INFO("%d",pose);
}
void robotTool::setGripperForce(const int force){
    ROS_INFO("%d",force);
}
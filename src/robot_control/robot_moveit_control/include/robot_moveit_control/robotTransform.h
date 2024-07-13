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

class robotTransform
{
private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    static tf2_ros::TransformBroadcaster broadcaster;
    static tf2_ros::StaticTransformBroadcaster stbroadcaster;

public:
    robotTransform();
    ~robotTransform();
    // 发布静态TF的函数
    void publishStaticTF(const geometry_msgs::Pose &p)
    {
        ROS_INFO("Publish TF task");

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
        stbroadcaster.sendTransform(ts);
    }
    geometry_msgs::Pose transP(geometry_msgs::Pose p)
    {
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
    void publishStaticTFwithRot(const geometry_msgs::Pose &p)
    {
        ROS_INFO("Publish TF Pose");
        // 创建坐标系信息
        geometry_msgs::TransformStamped ts;
        // 设置头信息
        ts.header.seq = 100;
        ts.header.stamp = ros::Time::now();
        ts.header.frame_id = "base_link_rm";
        // 设置子级坐标系
        ts.child_frame_id = "Pose";
        // 设置子级相对于父级的偏移量
        ts.transform.translation.x = p.position.x;
        ts.transform.translation.y = p.position.y;
        ts.transform.translation.z = p.position.z;
        ts.transform.rotation = p.orientation;
        // 广播器发布坐标系信息
        stbroadcaster.sendTransform(ts);
        ros::Duration(1).sleep();
    }
    void publishGraspPoses(const std::vector<vgn::GraspConfig> &grasps)
    {
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

geometry_msgs::Pose calculateTargetPose(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &trans_pose)
{
	tf2::Transform target_tf;
	tf2::fromMsg(target_pose, target_tf);

	tf2::Transform relative_tf;
	tf2::fromMsg(trans_pose, relative_tf);

	tf2::Transform end_target_tf = target_tf * relative_tf.inverse();

	geometry_msgs::Pose end_target_pose;
	tf2::toMsg(end_target_tf, end_target_pose);
	return end_target_pose;
}
geometry_msgs::Pose calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform)
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
geometry_msgs::Pose transformPose(const geometry_msgs::Pose &pose, const tf2::Transform &transform)
{
	tf2::Transform pose_transform;
	tf2::fromMsg(pose, pose_transform);

	tf2::Transform result_transform = transform * pose_transform;
	geometry_msgs::Pose result_pose;
	tf2::toMsg(result_transform, result_pose);
	return result_pose;
}

geometry_msgs::Pose moveFromPose(const geometry_msgs::Pose &pose, double distance)
{
	tf2::Transform pose_transform;
	tf2::fromMsg(pose, pose_transform);
	// 构造一个后退的变换矩阵
	tf2::Vector3 backward_vector(0.0, 0.0, distance); // 假设后退方向沿Z轴
	tf2::Vector3 transformed_vector = pose_transform.getBasis() * backward_vector;

	tf2::Transform back_transform;
	back_transform.setIdentity();
	back_transform.setOrigin(transformed_vector);

	// 将目标位置应用后退变换矩阵
	return transformPose(pose, back_transform);
}
};

robotTransform::robotTransform(/* args */)
{
    tfListener = new tf2_ros::TransformListener(tfBuffer);
}

robotTransform::~robotTransform()
{
}

#endif
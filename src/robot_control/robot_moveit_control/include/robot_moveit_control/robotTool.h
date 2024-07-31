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
#include <vgn/GetMapCloud.h>
#include <vgn/GetSceneCloud.h>
#include <vgn/PredictGrasps.h>
#include <vgn/GraspConfig.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
class robotTool
{
private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::StaticTransformBroadcaster stbroadcaster;
    ros::ServiceClient resetmap, toggle, get_scene_cloud_client, get_map_cloud_client, predict_grasps_client;
    ros::NodeHandle nh;

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
                            geometry_msgs::Pose transformed_pose = transPose(grasp.pose);
                            ret.push_back(transformed_pose);
                            valid_grasps.push_back(grasp); // 如果夹角在-45到45度之间，将该姿态加入有效姿态列表
                        }
                    }
                }
                publishGraspPoses(valid_grasps);
            }
            else
            {
                ROS_ERROR("Failed to call service PredictGrasps");
            }
        }
    }
};

robotTool::robotTool()
{
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    resetmap = nh.serviceClient<std_srvs::Empty>("reset_map");
    toggle = nh.serviceClient<std_srvs::SetBool>("toggle_integration");
    get_scene_cloud_client = nh.serviceClient<vgn::GetSceneCloud>("get_scene_cloud");
    get_map_cloud_client = nh.serviceClient<vgn::GetMapCloud>("get_map_cloud");
    predict_grasps_client = nh.serviceClient<vgn::PredictGrasps>("predict_grasps");
}

robotTool::~robotTool()
{
    delete tfListener;
}

#endif
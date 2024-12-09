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
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <rm_msgs/Modus_Write_Registers.h>
#include <iomanip>
#include <sstream>
#include <cstdint>

#define _Gripper_Address 0x01
#define _Set_Gripper_Position 0x9C40
#define _Set_Gripper_Force 0x9C41
class robotTool
{
private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener *tfListener;
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::StaticTransformBroadcaster stbroadcaster;
    ros::Publisher command_pub, set_ModusMod_pub;
    ros::NodeHandle nh;

public:
    robotTool();
    ~robotTool();
    void publishStaticTF(const geometry_msgs::Pose &p, std::string name = "task");
    geometry_msgs::Pose transPose(geometry_msgs::Pose p, std::string source_frame = "task", std::string target_frame = "base_link_rm");
    void publishStaticTFwithRot(const geometry_msgs::Pose &p, std::string name = "pose");
    geometry_msgs::Pose calculateTargetPose(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &trans_pose);
    geometry_msgs::Pose calculateTargetTransform(const geometry_msgs::Pose &target_pose, const geometry_msgs::Transform &relative_transform);
    geometry_msgs::Pose moveFromPose(const geometry_msgs::Pose &pose, double distance);
    double round(double num, int exponent);
    double degreesToRadians(double degrees);
    geometry_msgs::TransformStamped lookupTransform(std::string source_frame = "task", std::string target_frame = "base_link_rm")
    {
        return tfBuffer.lookupTransform(source_frame, target_frame, ros::Time(0));
    }
    void setModusMod()
    {
        std_msgs::Empty msg;
        set_ModusMod_pub.publish(msg);
    }
    void publishCommand(uint16_t address, uint8_t num, const std::vector<uint16_t> &data)
    {
        rm_msgs::Modus_Write_Registers msg;
        msg.device = _Gripper_Address;
        if (address == 1)
        {
            msg.address = _Set_Gripper_Position;
        }
        else
        {
            msg.address = _Set_Gripper_Force;
        }
        msg.num = num;
        msg.data = data; // 将数据直接填充为消息的数组
        command_pub.publish(msg);
        ROS_INFO("Published command: device=%d, address=%d, num=%d", msg.device, msg.address, msg.num);
    }
    void setGripperPosition(const uint16_t &data)
    {
        std::vector<uint16_t> dataArray;
        dataArray.push_back(0);
        dataArray.push_back(data);
        publishCommand(1, 1, dataArray);
    }
    void setGripperForce(const uint16_t &data)
    {
        std::vector<uint16_t> dataArray;
        dataArray.push_back(0);
        dataArray.push_back(data);
        publishCommand(2, 1, dataArray);
    }
};

robotTool::robotTool()
{
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    command_pub = nh.advertise<rm_msgs::Modus_Write_Registers>("/rm_driver/Write_Registers", 10);
    set_ModusMod_pub = nh.advertise<std_msgs::Empty>("/rm_driver/Set_Modbus_Mode", 10);
}

robotTool::~robotTool()
{
    delete tfListener;
}

#endif
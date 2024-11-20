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
#include <yd_msgs/Pose_Task.h>
#include <yd_msgs/MoveGlobalTargetAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/Call_TaskAction.h>
#include <robot_msgs/Objection_Detect.h>
#include <robotTool.h>

typedef actionlib::SimpleActionServer<robot_msgs::Call_TaskAction> Server;


int main(int argc, char *argv[])
{
    /* code */
    return 0;
}

#include<robotControl.h>

robotControl::robotControl(/* args */)
{
    std::string PLANNING_GROUP = "arm"; // 你的规划组名称
    moveit_server = new MoveitServer(PLANNING_GROUP);
    ros::CallbackQueue queue1;
    // ros::CallbackQueue queue2;
    // ros::CallbackQueue queue3;
    moveJ_pub = nh_.advertise<std_msgs::Float32MultiArray>("/robot/MoveJ_cmd", 20);
    moveP_pub = nh_.advertise<geometry_msgs::Pose>("/robot/MoveP_cmd", 20);
    moveL_pub = nh_.advertise<robot_msgs::MoveL_Data>("/robot/MoveL_cmd", 20);
    moveJ_sub = nh_.subscribe<std_msgs::Float32MultiArray>("/robot/MoveJ_cmd", 20, boost::bind(&robotControl::MoveJ_cmd_callback, this, _1));
    moveP_sub = nh_.subscribe<geometry_msgs::Pose>("/robot/MoveP_cmd", 20, boost::bind(&robotControl::MoveP_cmd_callback, this, _1));
    moveL_sub = nh_.subscribe<robot_msgs::MoveL_Data>("/robot/MoveL_cmd", 20, boost::bind(&robotControl::MoveL_cmd_callback, this, _1));
    nh_.setCallbackQueue(&queue1);
    ros::AsyncSpinner spinner(3,&queue1);
    spinner.start();
    
}

robotControl::~robotControl()
{
    delete this;
}
void robotControl::Close()
{
    this->moveit_server->stop();
}
void robotControl::MoveJ_cmd(const float pose[])
{
    std_msgs::Float32MultiArray msgs;
    for (int i = 0; i < 6; i++)
    {
        msgs.data.push_back(pose[i]);
    }
    this->moveJ_pub.publish(msgs);
    ros::spinOnce();
}
void robotControl::MoveJ_cmd(const std::vector<double> pose)
{
    std_msgs::Float32MultiArray msgs;
    for (int i = 0; i < pose.size(); i++)
    {
        msgs.data.push_back(pose[i]);
    }
    this->moveJ_pub.publish(msgs);
    ros::spinOnce();
}
void robotControl::MoveP_cmd(const double x, const double y, const double z)
{
    geometry_msgs::Pose target;
    target = this->moveit_server->setPoint(x, y, z);
    this->moveP_pub.publish(target);
    ros::spinOnce();
}
void robotControl::MoveP_cmd(const geometry_msgs::Pose target)
{
    this->moveP_pub.publish(target);
    ros::spinOnce();
}
void robotControl::MoveL_cmd(robot_msgs::MoveL_Data msgs)
{
    this->moveL_pub.publish(msgs);
    ros::spinOnce();
}
void robotControl::MoveJ_cmd_callback(const std_msgs::Float32MultiArrayConstPtr joint)
{
    std::vector<float> floatData = joint->data;

    // 将float类型数据转换为double类型，并存储到std::vector<double>中
    std::vector<double> doubleData;
    for (size_t i = 0; i < floatData.size(); ++i)
    {
        doubleData.push_back(static_cast<double>(floatData[i]));
    }

    this->moveit_server->move_j(doubleData, false);
}
void robotControl::MoveP_cmd_callback(const geometry_msgs::PoseConstPtr pose)
{
    geometry_msgs::Pose msg;
    msg.orientation = pose.get()->orientation;
    msg.position = pose.get()->position;
    this->moveit_server->move_p(msg, false);
}
void robotControl::MoveL_cmd_callback(const robot_msgs::MoveL_DataConstPtr msgs)
{
    std::vector<std::vector<double>> posees;
    for (int i = 0; i < msgs.get()->rowls; i++)
    {
        std::vector<double> temp;
        for (int j = 0; j < msgs.get()->cols; j++)
        {
            int k = (i * msgs.get()->cols) + j;
            temp.push_back(static_cast<double>(msgs.get()->points[k]));
        }
        posees.push_back(temp);
    }
    this->moveit_server->move_l(posees);
}
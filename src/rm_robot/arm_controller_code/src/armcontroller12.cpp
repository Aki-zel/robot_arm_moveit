#include <armcontroller.h>

ArmController::ArmController(std::string serverName, ros::NodeHandle nh)
    : server(nh, serverName, boost::bind(&ArmController::executeCB, this, _1),
             false)
{
    setlocale(LC_ALL, "");

    nh.param<std::string>("ip", this->ip, "192.168.0.18");
    // 话题方式处理
    // target_pub = nh.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 300);
    // pub_getArmStateTimerSwitch = nh.advertise<std_msgs::Bool>("/rm_driver/GetArmStateTimerSwitch", 200);
    // float min_interval = 0.02; // 透传周期,单位:秒
    // this->State_Timer = nh.createTimer(ros::Duration(min_interval), boost::bind(&ArmController::timer_callback, this));
    // .so API方式
    RM_API_Init(65, NULL);
    this->sockets = Arm_Socket_Start(strdup(this->ip.c_str()), 8080, 200);
    if (Arm_Socket_State(this->sockets) == 0)
    {
        ROS_INFO("--------------连接机械臂成功--------------\n");
        ROS_INFO("--------------IP : %s SOCKET : %d ---------\n", this->ip.c_str(), this->sockets);
        float a[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 180.0};
        // 机械臂初始化
        int ret = this->service.Service_Movej_Cmd(this->sockets, a, 25, 0, 1);
        ROS_INFO("--------------初始化机械臂--------------\n");
        if (ret == 0)
        {
            ROS_INFO("--------------初始化成功--------------\n");
        }
        else
        {
            ROS_INFO("--------------请检查--------------\n");
        }
    }
    else
    {
        ROS_INFO("--------------连接机械臂失败--------------\n");
        ROS_INFO("--------------请检查ip地址设置--------------\n");
    }
    this->statePub = nh.advertise<sensor_msgs::JointState>("arm_controller_joint_states", 300);
    server.start();
    std::thread publishThread(&ArmController::sentCurrentStateThread, this);
    publishThread.detach();
    std::thread sendGoaltraThread(&ArmController::sendGoaltra, this);
    sendGoaltraThread.detach();
    this->service.Service_Set_Gripper_Pick(this->sockets, 500, 50, 0);
    this->handserver = nh.subscribe<std_msgs::Bool>("hand", 200, boost::bind(&ArmController::handcallback, this, _1));
    ROS_INFO("--------------开启server--------------\n");
}
bool ArmController::linkToarm()
{
    RM_API_Init(65, NULL);
    this->sockets = Arm_Socket_Start(strdup(this->ip.c_str()), 8080, 200);
    if (Arm_Socket_State(this->sockets) == 0)
    {
        ROS_INFO("--------------连接机械臂成功--------------\n");
        ROS_INFO("--------------IP : %s SOCKET : %d ---------\n", this->ip.c_str(), this->sockets);
        return true;
    }
    else
    {
        ROS_INFO("--------------连接机械臂失败--------------\n");
        ROS_INFO("--------------请检查ip地址设置--------------\n");
        return false;
    }
}
void ArmController::handcallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg)
    {
        this->service.Service_Set_Gripper_Pick(this->sockets, 500, 50, 0);
    }
    else
    {
        this->service.Service_Set_Gripper_Release(this->sockets, 500, 0);
    }
}
ArmController::~ArmController()
{
    delete this;
}

void ArmController::sentCurrentStateThread()
{
    sensor_msgs::JointState jointstate;
    jointstate.header.frame_id = "dummy";
    jointstate.name.push_back("joint1");
    jointstate.name.push_back("joint2");
    jointstate.name.push_back("joint3");
    jointstate.name.push_back("joint4");
    jointstate.name.push_back("joint5");
    jointstate.name.push_back("joint6");
    float pos[6];
    ros::Rate rate(15);
    ROS_INFO("--------------开启publish--------------\n");
    while (ros::ok())
    {
        try
        {
            if (Arm_Socket_State(this->sockets) == 0)
            {
                int ret = this->service.Service_Get_Joint_Degree(this->sockets, pos);
                if (ret == 0)
                {
                    jointstate.header.stamp = ros::Time::now();
                    for (int i = 0; i < 6; i++)
                    {
                        pos[i] = pos[i] * PI / 180;
                    }
                    jointstate.position.assign(pos, pos + 6);
                    this->statePub.publish(jointstate);
                }
                rate.sleep();
                ros::spinOnce();
            }
            else
            {
                // ROS_INFO("jointstatePub Error:尝试重新连接机械臂");
                this->linkToarm();
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
}
void ArmController::sendGoaltra()
{
    float pos[6];
    while (ros::ok())
    {
        if (this->point_changed && this->ret.node != 0)
        {
            ROS_INFO("发送数据");
            try
            {
                if (Arm_Socket_State(this->sockets) == 0)
                {
                    for (int i = 1; i < this->ret.node; i++)
                    {
                        for (int j = 0; j < this->ret.link; j++)
                        {
                            pos[j] = (this->ret.joint[j].positions[i]) * 180 / PI;
                            // std::cout << pos[j] << "  ";
                        }
                        // std::cout << std::endl;
                        int ret = this->service.Service_Movej_CANFD(this->sockets, pos);
                        if (ret != 0)
                        {
                            ROS_INFO("检查机械臂状态");
                        }
                        ros::Duration(0.02).sleep();
                        // sleep(0.2);
                    }
                    this->ret.node = 0;
                    this->point_changed = false;
                }
                else
                {
                    ROS_INFO("sendGoaltra Error:尝试重新连接机械臂");
                    this->linkToarm();
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
    }
}
void ArmController::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goalPtr)
{
    try
    {
        this->ret.link = goalPtr->trajectory.joint_names.size();
        // 获取轨迹点的个数
        this->ret.node = goalPtr->trajectory.points.size();
        ROS_INFO("获得规划路径点，个数为： %d ", this->ret.node);
        std_msgs::Bool timerSwitch;
        this->point_changed = false;
        this->ret.time.clear();
        for (int j = 0; j < this->ret.link; j++)
        {
            this->ret.joint[j].accelerations.clear();
            this->ret.joint[j].positions.clear();
            this->ret.joint[j].velocities.clear();
        }
        for (int i = 0; i < this->ret.node; i++)
        {
            for (int j = 0; j < this->ret.link; j++)
            {
                this->ret.joint[j].positions.push_back(goalPtr->trajectory.points[i].positions[j]);
                this->ret.joint[j].velocities.push_back(goalPtr->trajectory.points[i].velocities[j]);
                this->ret.joint[j].velocities.push_back(goalPtr->trajectory.points[i].accelerations[j]);
            }
            this->ret.time.push_back(goalPtr->trajectory.points[i].time_from_start.toSec()); // 获得时间信息
        }
        int flag = 0;
        if (this->ret.node > 3) // 判断当moveit规划的路点数大于3时为有效规划并进行三次样条插值
        {
            cubicSpline spline;
            double max_time = this->ret.time[this->ret.node - 1];
            double rate = 0.02;
            ROS_INFO("规划所需最大时间 %f ", max_time);
            // this->ret.time.clear();
            RetData tempdata;
            int node = this->ret.node;
            std::vector<double> time = this->ret.time;
            for (int j = 0; j < this->ret.link; j++)
            {
                double x_out;
                double y_out;
                tempdata.time.clear();
                this->ret.time.clear();
                flag = 0;
                if (spline.loadData(time.data(), this->ret.joint[j].positions.data(), node, 0, 0, cubicSpline::BoundType_First_Derivative))
                {
                    x_out = -rate;
                    while (x_out < max_time)
                    {
                        x_out += rate;
                        spline.getYbyX(x_out, y_out);
                        tempdata.time.emplace_back(x_out); // 将新的时间存储，只需操作一次即可
                        tempdata.joint[j].positions.push_back(y_out);
                        tempdata.joint[j].velocities.push_back(spline.getVel());
                        tempdata.joint[j].accelerations.push_back(spline.getAcc());
                    }
                    this->ret.joint[j] = tempdata.joint[j];
                    this->ret.time = tempdata.time;
                    this->ret.node = tempdata.time.size();
                }
            }
            flag = 1;
            ROS_INFO("插补后路点数为 %d.", this->ret.node);
        }
        else
        {
            flag = 1;
        }
        if (flag == 1)
        {
            // 话题方式处理
            // timerSwitch.data = true;
            // pub_getArmStateTimerSwitch.publish(timerSwitch);
            // this->current = 0;
            // this->point_changed = true;
            // while (point_changed)
            // {
            //     if (server.isPreemptRequested() || !ros::ok())
            //     {
            //         ROS_INFO("************************Action Server: Preempted");
            //         this->point_changed = false;
            //         server.setPreempted();
            //         timerSwitch.data = false;
            //         pub_getArmStateTimerSwitch.publish(timerSwitch);
            //         return;
            //     }
            // }
            // .so API方式
            this->point_changed = true;
            // this->sendGoaltra();
            while (this->point_changed)
            {
                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("********Action Server: Preempted********");
                    this->point_changed = false;
                    server.setPreempted();
                    return;
                }
            }
        }
        else
        {
            ROS_INFO("处理失败");
        }
        server.setSucceeded();
        // 话题方式处理
        // timerSwitch.data = false;
        // pub_getArmStateTimerSwitch.publish(timerSwitch);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);

    ArmController contorller("arm_controller/follow_joint_trajectory", nh);
    // std::thread serverThread([&contorller]()
    //                          { ros::spin(); });

    // // 等待CycloidServer线程结束
    // serverThread.join();
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

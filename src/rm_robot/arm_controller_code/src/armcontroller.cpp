#include <armcontroller.h>

ArmController::ArmController(std::string serverName, ros::NodeHandle nh)
    : server(nh, serverName, boost::bind(&ArmController::executeCB, this, _1),
             false)
{
    setlocale(LC_ALL, "");
    server.start();
    target_pub = nh.advertise<rm_msgs::JointPos>("/rm_driver/JointPos", 300);
    pub_getArmStateTimerSwitch = nh.advertise<std_msgs::Bool>("/rm_driver/GetArmStateTimerSwitch", 200);

    float min_interval = 0.02; // 透传周期,单位:秒
    this->State_Timer = nh.createTimer(ros::Duration(min_interval), boost::bind(&ArmController::timer_callback, this));
}

ArmController::~ArmController()
{
}
void ArmController::timer_callback()
{
    rm_msgs::JointPos msg;

    if (this->point_changed && this->ret.node != 0)
    {
        if (this->current <= this->ret.node)
        {
            for (int j = 0; j < this->ret.link; j++)
            {
                msg.joint[j] = this->ret.joint[j].positions[current];
            }
            target_pub.publish(msg);
            this->current++;
        }
    }
    else
    {
        this->current = 0;
        this->ret.node = 0;
        this->point_changed = false;
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
        for (int i = 0; i < this->ret.node; i++)
        {
            for (int j = 0; j < this->ret.link; j++)
            {
                this->ret.joint[j].positions.emplace_back(goalPtr->trajectory.points[i].positions[j]);
                this->ret.joint[j].velocities.emplace_back(goalPtr->trajectory.points[i].velocities[j]);
                this->ret.joint[j].velocities.emplace_back(goalPtr->trajectory.points[i].accelerations[j]);
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
            this->ret.time.clear();
            RetData tempdata;
            for (int j = 0; j < this->ret.link; j++)
            {
                double x_out;
                double y_out;
                tempdata.time.clear();
                flag = 0;
                if (spline.loadData(this->ret.time.data(), this->ret.joint[j].positions.data(), this->ret.node, 0, 0, cubicSpline::BoundType_First_Derivative))
                {
                    x_out = -rate;
                    while (x_out < max_time)
                    {
                        x_out += rate;
                        spline.getYbyX(x_out, y_out);
                        tempdata.time.emplace_back(x_out); // 将新的时间存储，只需操作一次即可
                        tempdata.joint[j].positions.emplace_back(y_out);
                        tempdata.joint[j].velocities.emplace_back(spline.getVel());
                        tempdata.joint[j].accelerations.emplace_back(spline.getAcc());
                    }
                    this->ret.joint[j] = tempdata.joint[j];
                    this->ret.time = tempdata.time;
                    this->ret.node = tempdata.time.size();
                    flag = 1;
                }
            }
            ROS_INFO("插补后路点数为 %d.", this->ret.node);
        }
        else
        {
            flag = 1;
        }
        if (flag == 1)
        {
            timerSwitch.data = true;
            pub_getArmStateTimerSwitch.publish(timerSwitch);
            this->current = 0;
            point_changed = true;
            while (point_changed)
            {
                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("************************Action Server: Preempted");
                    point_changed = false;
                    server.setPreempted();
                    timerSwitch.data = false;
                    pub_getArmStateTimerSwitch.publish(timerSwitch);
                    return;
                }
            }
        }
        else
        {
            ROS_INFO("处理失败");
        }
        server.setSucceeded();
        timerSwitch.data = false;
        pub_getArmStateTimerSwitch.publish(timerSwitch);
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
    // ros::AsyncSpinner spinner(2);
    // spinner.start();
<<<<<<< HEAD:src/rm_robot/arm_controller_code/src/armcontroller.cpp
    ArmController contorller("arm_controller/follow_joint_trajectory", nh);
=======
    // ArmController contorller("arm_controller/follow_joint_trajectory", nh);
    
    ArmController contorller("rm_group/follow_joint_trajectory", nh);
>>>>>>> b9028ea4760805b61117aa37a9889ad1b81c2ec8:src/rm_robot/arm_controller_code/src/armcontroller11.cpp
    std::thread serverThread([&contorller]()
                             { ros::spin(); });

    // 等待CycloidServer线程结束
    serverThread.join();
    // ros::spin();
    return 0;
}
